"""Absolute ground control from PAINTED ROAD MARKINGS, to settle where the bays are.

    python vision/diag/paint_ground_control.py <stills_dir> [--every N]
           [--search 15] [--cell 0.25] [--self-test] [--dump DIR]

`real_align.py` draws the bays and lets the eye judge; `ref_align.py` adds OSM
buildings as a control. Neither settles the question this exists for, for two
reasons: a building is TALL, so its roof leans away from its footprint at 30 m
and a metre of apparent offset may be parallax rather than error; and the eye
cannot separate "the pose is shifted" from "the bay polygons are drawn shifted".

Painted markings fix both. Paint is **on the ground plane** -- the very plane
`unproject()` assumes -- so it has no parallax, and it is the same physical thing
the bay dataset claims to describe. So:

    unproject every paint pixel of every frame -> a paint map in ENU metres
    raster the Sofiaplan bay OUTLINES on the same grid
    slide one over the other and find the offset that best explains the paint

A peak at (0, 0) means the bays are where the paint is and the georeferencing is
right. A peak at (dx, dy) is a MEASURED offset -- and because the paint map is
built from the pose while the bay raster is not, that offset is exactly the
disagreement between our georeferencing and Sofiaplan's geometry, with no appeal
to anybody's eye.

**What it cannot do alone:** it measures the offset, it does not attribute it. A
constant GPS bias on the flight and a mis-drawn street both peak away from zero.
Attribution needs the peak compared ACROSS flights and streets -- which is why
the report prints the peak's sharpness and the paint volume behind it, not just
the winning vector.

`--self-test` is not optional before believing a number: it shifts the bay raster
by known vectors and checks the search recovers them.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cameras                                                   # noqa: E402
import check_nadir                                               # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

# Paint is bright, unsaturated, and sits on much darker asphalt. These are
# deliberately loose: the correlation does not need clean paint, it needs paint
# to outnumber whatever else is bright in the same places, and false pixels that
# are not aligned with anything add a flat background rather than a rival peak.
V_MIN = 160          # HSV value: paint is bright
S_MAX = 55           # HSV saturation: paint is grey/white, foliage and cars are not
TOPHAT_K = 31        # px, wider than any marking (~10 px) and narrower than a car
TOPHAT_MIN = 30      # how far above its own surface a marking must stand
BLOB_ERODE = 23      # px: anything still solid after this is a roof or a car, not a line
BLOB_GROW = 41       # px: grow those blobs back over their full extent before removing
MIN_LEN = 110        # px, ~1.3 m -- shorter "markings" are texture or dapple
MIN_ELONG = 4.0      # length / width of the bounding box
FILL_MAX = 0.45      # a line fills little of its bounding box; a patch fills most


def paint_mask(img):
    """Binary mask of probable road paint in a BGR frame.

    Brightness and low saturation alone are not enough, and the first version was
    useless because of it: a **white car roof is bright, grey and large**, and so
    is a rendered-metal balcony or a sunlit pale roof -- they swamped the actual
    markings, and they are exactly the objects a parking bay is drawn around, so
    their false paint would have biased the correlation towards the cars rather
    than the paint.

    What separates them is not colour but SHAPE: a road marking is a thin line
    (~0.12 m, about 10 px at this GSD) while a car or a roof is a blob metres
    across. Eroding with a kernel wider than any line erases every marking and
    leaves the blobs; dilating those back and subtracting them keeps the lines
    and drops the blobs.
    """
    import cv2
    import numpy as np
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]

    # 1. White top-hat: bright structures NARROWER than the kernel survive, the
    #    surface they sit on does not. This is the selective step -- a plain
    #    brightness threshold marked 2.9% of the frame (foliage highlights,
    #    pavement texture, kerbs) and produced a paint "map" that was a solid
    #    blob, against which a correlation peak means nothing.
    th = cv2.morphologyEx(v, cv2.MORPH_TOPHAT,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                    (TOPHAT_K, TOPHAT_K)))
    m = ((th >= TOPHAT_MIN) & (hsv[:, :, 1] <= S_MAX) & (v >= V_MIN)).astype(np.uint8)

    # 2. Drop blobs. A car body or a pale roof is bright and grey like paint;
    #    what separates them is shape, so erase anything still solid after an
    #    erosion wider than any marking, grow it back, and subtract.
    blob = cv2.erode(m, np.ones((BLOB_ERODE, BLOB_ERODE), np.uint8))
    blob = cv2.dilate(blob, np.ones((BLOB_GROW, BLOB_GROW), np.uint8))
    m = (m & (1 - blob)).astype(np.uint8)

    # 3. Keep only ELONGATED components. This is the step that has to defeat the
    #    real adversary on this footage: a midday sun through summer canopy
    #    scatters the ground with dappled highlights that are bright, grey and
    #    thin -- indistinguishable from paint by tone, and numerous enough to
    #    bury it. What they are not is long and straight. A road marking is
    #    metres long with a high length-to-width ratio; a sun fleck is a blob.
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    keep = np.zeros_like(m)
    for i in range(1, n):
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        long_side, short_side = max(w, h), max(1, min(w, h))
        if long_side < MIN_LEN or long_side / short_side < MIN_ELONG:
            continue
        # a solid rectangle is not a line: paint fills only a fraction of its box
        if area > FILL_MAX * w * h:
            continue
        keep[lab == i] = 1
    return keep.astype(np.uint8)


def bay_outline_points(bays, step=0.30):
    """Points along every bay's OUTLINE (not its interior), in ENU metres."""
    pts = []
    for b in bays:
        ring = b["ring"]
        for i in range(len(ring)):
            x0, y0 = ring[i]
            x1, y1 = ring[(i + 1) % len(ring)]
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
            for k in range(n):
                t = k / n
                pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return pts


def rasterise(pts, x0, y0, nx, ny, cell):
    import numpy as np
    g = np.zeros((ny, nx), dtype=np.float32)
    for x, y in pts:
        i = int((x - x0) / cell)
        j = int((y - y0) / cell)
        if 0 <= i < nx and 0 <= j < ny:
            g[j, i] = 1.0
    return g


def search(paint, bay, cell, reach):
    """Best (di, dj) shift of the BAY raster, by overlap with the paint raster."""
    import numpy as np
    r = int(round(reach / cell))
    best, scores = None, {}
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            b = np.roll(np.roll(bay, dj, axis=0), di, axis=1)
            s = float((b * paint).sum())
            scores[(di, dj)] = s
            if best is None or s > best[0]:
                best = (s, di, dj)
    return best, scores


def self_test(bay, cell, reach):
    """Recover known shifts, using the bays themselves as a synthetic paint map."""
    import cv2
    import numpy as np
    print("\nself-test -- shift the bay raster by a known vector, recover it:")
    ok = True
    synth = cv2.GaussianBlur(bay.copy(), (0, 0), 1.5)
    for tdx, tdy in [(0.0, 0.0), (2.0, 0.0), (-3.0, 4.0), (6.0, -5.0)]:
        shifted = np.roll(np.roll(bay, int(round(tdy / cell)), axis=0),
                          int(round(tdx / cell)), axis=1)
        (s, di, dj), _ = search(synth, shifted, cell, reach)
        gx, gy = -di * cell, -dj * cell
        err = math.hypot(gx - tdx, gy - tdy)
        good = err <= cell * 1.5
        ok &= good
        print(f"  {'ok  ' if good else 'FAIL'} injected ({tdx:+.1f}, {tdy:+.1f}) -> "
              f"recovered ({gx:+.2f}, {gy:+.2f}), error {err:.2f} m")
    print("  self-test PASSED" if ok
          else "  self-test FAILED -- do not trust the number below")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--search", type=float, default=15.0, help="+/- metres")
    ap.add_argument("--cell", type=float, default=0.25)
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--dump", help="write the paint map and bay outline as PNGs here")
    a = ap.parse_args()

    import cv2
    import numpy as np

    cam = cameras.get(a.cam)
    poses = [p for p in json.load(open(os.path.join(a.stills, "poses.json"),
                                      encoding="utf-8")) if "yaw" in p and "x" in p]
    poses = poses[::a.every]
    bays = so.load_all_bays()

    # Cars must be masked OUT, and this is a methodological requirement rather
    # than a tidy-up. Erosion removes a car's solid body but its bright OUTLINE
    # survives, and a car outline is a bay-sized rectangle lying exactly where
    # the question is. Left in, it would pull the correlation towards the cars --
    # which is the very conclusion under test, so the method would be assuming
    # its answer. The detector already knows where the cars are, so use it.
    cars = {}
    dets_dir = os.path.join(a.stills, "detect")
    if os.path.isdir(dets_dir):
        for fn in sorted(os.listdir(dets_dir)):
            if fn.startswith("detections_") and fn.endswith(".json"):
                for r in json.load(open(os.path.join(dets_dir, fn), encoding="utf-8")):
                    cars.setdefault(r["file"], []).extend(d["box"] for d in r["dets"])
                break
    print(f"car masking: {'on, ' + str(sum(len(v) for v in cars.values())) + ' boxes'
                          if cars else 'OFF -- no cached detections found'}")

    pts, used, skipped = [], 0, 0
    for pose in poses:
        img = cv2.imread(os.path.join(a.stills, pose["file"]))
        if img is None:
            continue
        if check_nadir.is_oblique(img)[2]:
            skipped += 1
            continue
        m = paint_mask(img)
        for bx in cars.get(pose["file"], []):
            x0b, y0b, x1b, y1b = (int(round(c)) for c in bx)
            pad_px = 12                       # cover the box's own bright edge
            m[max(0, y0b - pad_px):y1b + pad_px,
              max(0, x0b - pad_px):x1b + pad_px] = 0
        ys, xs = np.nonzero(m)
        if len(xs) > 60000:              # thin dense frames evenly, keep the shape
            k = len(xs) // 60000 + 1
            xs, ys = xs[::k], ys[::k]
        for u, v in zip(xs.tolist(), ys.tolist()):
            pts.append(so.unproject(float(u), float(v), pose, cam=cam))
        used += 1
    print(f"{used} nadir frames used ({skipped} oblique skipped), "
          f"{len(pts)} paint points unprojected")
    if not pts:
        sys.exit("no paint found -- loosen V_MIN/LOCAL_GAIN or check the frames")

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad = a.search + 5
    x0, y0 = min(xs) - pad, min(ys) - pad
    nx = int((max(xs) + pad - x0) / a.cell) + 1
    ny = int((max(ys) + pad - y0) / a.cell) + 1
    paint = rasterise(pts, x0, y0, nx, ny, a.cell)
    # Blur slightly: a 0.25 m cell is finer than the paint is straight, and a hard
    # raster would make the score jitter on sub-cell alignment.
    paint = cv2.GaussianBlur(paint, (0, 0), 1.5)

    near = [b for b in bays
            if x0 - 5 < b["cx"] < x0 + nx * a.cell + 5
            and y0 - 5 < b["cy"] < y0 + ny * a.cell + 5]
    print(f"paint map {nx}x{ny} cells of {a.cell} m, {len(near)} bays in the box")
    if not near:
        sys.exit("no bays overlap the flown area")
    bay = rasterise(bay_outline_points(near), x0, y0, nx, ny, a.cell)

    if a.self_test:
        self_test(bay, a.cell, a.search)

    (best, di, dj), scores = search(paint, bay, a.cell, a.search)
    dx, dy = di * a.cell, dj * a.cell
    zero = scores[(0, 0)]
    vals = sorted(scores.values())
    med = vals[len(vals) // 2]
    print(f"\nbest offset applied to the BAY polygons: ({dx:+.2f}, {dy:+.2f}) m "
          f"[{math.hypot(dx, dy):.2f} m]")
    print(f"  overlap score at that offset {best:.0f} | at (0,0) {zero:.0f} "
          f"| median over the search {med:.0f}")
    if best > 0:
        print(f"  improvement over no shift: {100 * (best - zero) / best:.1f}% "
              f"of the peak")
        print(f"  peak sharpness (peak/median): {best / max(med, 1e-6):.2f}x")
    print("\n  A peak at (0,0) means the bays sit on the paint and the "
          "georeferencing is sound.\n  A peak elsewhere is a MEASURED "
          "disagreement -- but it does not say whose:\n  compare it across "
          "flights and streets before attributing it.")

    if a.dump:
        os.makedirs(a.dump, exist_ok=True)
        def rgb(bays_layer):
            vis = np.zeros((ny, nx, 3), dtype=np.uint8)
            vis[:, :, 1] = np.clip(paint * 255 / max(paint.max(), 1e-6), 0, 255)
            vis[:, :, 2] = np.clip(bays_layer * 255, 0, 255)
            return cv2.flip(vis, 0)          # ENU y is up, image rows go down
        cv2.imwrite(os.path.join(a.dump, "paint_vs_bays.png"), rgb(bay))
        cv2.imwrite(os.path.join(a.dump, "paint_vs_bays_shifted.png"),
                    rgb(np.roll(np.roll(bay, dj, axis=0), di, axis=1)))
        print(f"\n  -> {a.dump}/paint_vs_bays.png (GREEN paint, RED bays) "
              f"and _shifted.png")


if __name__ == "__main__":
    main()
