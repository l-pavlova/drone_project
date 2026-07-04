#!/usr/bin/env python3
"""
PARKDRONE vision stage v1: per-bay occupancy from captured nadir frames.

Georeferencing-first approach: bay polygons (data/block_bays.geojson) and the
drone pose per frame (sim/output/<world>/poses.json) live in the same pinned
ENU frame as the world, so each bay can be PROJECTED into each frame. For
every bay we pick the best view (bay fully inside the frame, closest to the
image centre), classify the bay crop occupied/free, and score the predictions
against sim/worlds/<world>.ground_truth.json.

    python score_occupancy.py [world_name]      # default fmi_block_4st

Writes sim/output/<world>/occupancy_results.json and annotated debug frames to
sim/output/<world>/debug/. The v1.5 classifier is deliberately simple (painted
bays are near-white; a parked car covers the paint): paint/dark fractions plus
brightness and texture thresholds, evaluated on the lengthwise CORE of the bay
(a neighbour's car can only overhang past the bay's short ends, so the core
stays clean on a free bay). Views that show only part of the bay still count
as long as enough of the core is visible. Swap `classify()` for a learned
model later — projection, view selection and scoring stay unchanged.
"""
import json, math, os, sys, collections
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")

WORLD = sys.argv[1] if len(sys.argv) > 1 else "fmi_block_4st"
OUT = os.path.join(ROOT, "sim", "output", WORLD)
GT_FILE = os.path.join(ROOT, "sim", "worlds",
                       "ground_truth.json" if WORLD == "fmi_block"
                       else f"{WORLD}.ground_truth.json")
BAYS = os.path.join(ROOT, "data", "block_bays.geojson")

# camera intrinsics of the Mavic2Pro proto camera (see parkdrone.py startup log)
IMG_W, IMG_H = 400, 240
FOV = 0.7854                     # horizontal, radians
EDGE_MARGIN = 2                  # px: ignore pixels this close to the frame edge
INNER = 0.78                     # sample only the inner fraction of the bay
                                 # (excludes the painted outline + neighbours,
                                 # but keeps enough of a car's dark glass/wheels)
CORE_LEN = 0.55                  # central fraction of the bay's LONG axis used
                                 # for classification (immune to overhang from
                                 # the adjacent bays, which enters at the ends)
MIN_VIS = 0.70                   # a view is usable if at least this fraction of
                                 # the core area is inside the frame

# MUST match ORIGIN in sim/generate_world.py (the project's shared ENU frame)
ORIGIN = (42.6747105, 23.3298956)
MLAT = 111320.0
MLON = 111320.0 * math.cos(math.radians(ORIGIN[0]))


def to_enu(lon, lat):
    return (lon - ORIGIN[1]) * MLON, (lat - ORIGIN[0]) * MLAT


def project(px, py, pose):
    """Ground ENU point -> pixel (u, v) for a nadir camera at the pose.
    Image up = drone heading; square pixels; horizontal FOV across IMG_W."""
    k = IMG_W / (2.0 * pose["alt"] * math.tan(FOV / 2.0))
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    rx, ry = px - pose["x"], py - pose["y"]
    along = c * rx + s * ry          # + ahead of the drone
    cross = -s * rx + c * ry         # + left of the drone
    return IMG_W / 2.0 - cross * k, IMG_H / 2.0 - along * k


def load_bays():
    """Bays that exist in this world's ground truth, as ENU polygons."""
    gt = json.load(open(GT_FILE, encoding="utf-8"))
    feats = json.load(open(BAYS, encoding="utf-8"))["features"]
    bays = []
    for f in feats:
        bid = str(f["properties"].get("id"))
        if bid not in gt:
            continue
        ring = [to_enu(lon, lat) for lon, lat in f["geometry"]["coordinates"][0][:-1]]
        bays.append({"id": bid, "ring": ring, "occupied": gt[bid],
                     "cx": sum(p[0] for p in ring) / len(ring),
                     "cy": sum(p[1] for p in ring) / len(ring)})
    return bays


def shrink(ring_px, frac):
    cx = sum(p[0] for p in ring_px) / len(ring_px)
    cy = sum(p[1] for p in ring_px) / len(ring_px)
    return [(cx + (x - cx) * frac, cy + (y - cy) * frac) for x, y in ring_px]


def shrink_long(ring_px, frac):
    """Shrink a 4-corner ring along its LONG axis only (keeps full width)."""
    t = (1.0 - frac) / 2.0
    if (math.dist(ring_px[0], ring_px[1]) >= math.dist(ring_px[1], ring_px[2])):
        pairs = [(0, 1), (1, 0), (2, 3), (3, 2)]    # long edges: 0-1 and 2-3
    else:
        pairs = [(0, 3), (1, 2), (2, 1), (3, 0)]    # long edges: 1-2 and 3-0
    return [(ring_px[a][0] + (ring_px[b][0] - ring_px[a][0]) * t,
             ring_px[a][1] + (ring_px[b][1] - ring_px[a][1]) * t)
            for a, b in pairs]


def poly_area(poly):
    return 0.5 * abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2)
                         in zip(poly, poly[1:] + poly[:1])))


def region_stats(img_arr, poly):
    """Pixel stats over a polygon, clipped to the frame. Returns (stats, vis)
    where vis is the fraction of the polygon area that landed inside the frame
    (stats is None if too few pixels are visible to measure)."""
    full = poly_area(poly)
    if full < 12:
        return None, 0.0
    mask = Image.new("1", (IMG_W, IMG_H), 0)
    ImageDraw.Draw(mask).polygon(poly, fill=1)
    m = np.array(mask, dtype=bool)
    m[:EDGE_MARGIN, :] = m[-EDGE_MARGIN:, :] = False
    m[:, :EDGE_MARGIN] = m[:, -EDGE_MARGIN:] = False
    vis = min(m.sum() / full, 1.0)
    if m.sum() < 12:
        return None, vis
    px = img_arr[m].astype(np.float32)              # N x 3 (RGB)
    mx, mn = px.max(axis=1), px.min(axis=1)
    # "paint" pixels: bright and non-chromatic, like the white bay marking
    paint = (mn > 140) & ((mx - mn) < 45)
    # darkness fraction: glass, wheels, shadows - things paint never has
    dark = (px.mean(axis=1) < 90)
    return {"paint_frac": float(paint.mean()),
            "dark_frac": float(dark.mean()),
            "brightness": float(px.mean()),
            "std": float(px.std())}, vis


def bay_features(img_arr, ring_px):
    """Stats over the bay's inner region and its lengthwise core. Returns None
    unless enough of the CORE is visible in this frame to classify from."""
    inner_poly = shrink(ring_px, INNER)
    core, core_vis = region_stats(img_arr, shrink_long(inner_poly, CORE_LEN))
    if core is None or core_vis < MIN_VIS:
        return None
    feat = {"core_" + k: v for k, v in core.items()}
    inner, _ = region_stats(img_arr, inner_poly)    # reporting/debug only
    feat.update(inner or {})
    feat["vis"] = round(core_vis, 2)
    return feat


# calibrated on fmi_block_4st against ground truth (v1.5; a learned classifier
# can replace this without touching projection/view selection/scoring)
T_PAINT = 0.70
T_DARK = 0.04
T_BRIGHT = 225                   # free paint is ~239 under the sim's uniform
T_STD = 15                       # light and dead flat; any roof is darker/noisier


def classify(feat):
    """v1.5 heuristic on the bay CORE: a free core is uniform bright paint; a
    car covers the paint, shows dark glass/wheels, or (white-on-white case) an
    off-white textured roof. Ends of the bay are excluded so a neighbour's
    overhang cannot fake occupancy."""
    return (feat["core_paint_frac"] < T_PAINT or feat["core_dark_frac"] > T_DARK
            or feat["core_brightness"] < T_BRIGHT or feat["core_std"] > T_STD)


def main():
    poses = json.load(open(os.path.join(OUT, "poses.json"), encoding="utf-8"))
    bays = load_bays()
    frames = {}                                     # lazy-loaded image arrays

    def frame_arr(i):
        if i not in frames:
            p = os.path.join(OUT, f"frame_{i:03d}.png")
            frames[i] = np.array(Image.open(p).convert("RGB"))
        return frames[i]

    # candidate views per bay: bay bbox overlaps the frame at all (whether the
    # view is actually usable — enough of the core visible — is decided by
    # bay_features when the vote is computed)
    views = collections.defaultdict(list)          # bay id -> [(off, pose, ring_px)]
    for pose in poses:
        for b in bays:
            ring_px = [project(x, y, pose) for x, y in b["ring"]]
            us, vs = [p[0] for p in ring_px], [p[1] for p in ring_px]
            if max(us) < 0 or min(us) >= IMG_W or max(vs) < 0 or min(vs) >= IMG_H:
                continue
            off = math.hypot(sum(us) / 4 - IMG_W / 2, sum(vs) / 4 - IMG_H / 2)
            views[b["id"]].append((off, pose, ring_px))

    # majority vote across views (a neighbour's overhang or a glare only shows
    # from some angles); fuller views outrank partial ones, then ties go to the
    # view nearest the image centre
    results, uncovered = {}, []
    for b in bays:
        votes = []                                  # (vis, off, pred, pose_i, feat)
        for off, pose, ring_px in views.get(b["id"], []):
            feat = bay_features(frame_arr(pose["i"]), ring_px)
            if feat is not None:
                votes.append((feat["vis"], off, classify(feat), pose["i"], feat))
        if not votes:
            uncovered.append(b["id"])
            continue
        votes.sort(key=lambda v: (-v[0], v[1]))
        n_occ = sum(1 for v in votes if v[2])
        pred = n_occ * 2 > len(votes)               # strict majority sees a car
        vis, off, _, pose_i, feat = votes[0]        # best view, for reporting
        results[b["id"]] = {"pred": pred, "gt": b["occupied"], "frame": pose_i,
                            "views": len(votes), "votes_occupied": n_occ,
                            "center_off_px": round(off, 1), **feat}

    # ---- metrics
    tp = sum(1 for r in results.values() if r["pred"] and r["gt"])
    tn = sum(1 for r in results.values() if not r["pred"] and not r["gt"])
    fp = sum(1 for r in results.values() if r["pred"] and not r["gt"])
    fn = sum(1 for r in results.values() if not r["pred"] and r["gt"])
    n = len(results)
    print(f"world {WORLD}: {len(bays)} bays, {n} classified, {len(uncovered)} uncovered")
    print(f"confusion: TP={tp} TN={tn} FP={fp} FN={fn}")
    if n:
        print(f"accuracy {100*(tp+tn)/n:.1f}%   "
              f"precision {100*tp/(tp+fp):.1f}%   recall {100*tp/(tp+fn):.1f}%"
              if (tp+fp) and (tp+fn) else f"accuracy {100*(tp+tn)/n:.1f}%")
    wrong = {k: r for k, r in results.items() if r["pred"] != r["gt"]}
    if wrong:
        print("misclassified:")
        for k, r in sorted(wrong.items()):
            print(f"  bay {k}: gt={'occ' if r['gt'] else 'free'} "
                  f"core paint={r['core_paint_frac']:.2f} dark={r['core_dark_frac']:.2f} "
                  f"bright={r['core_brightness']:.0f} std={r['core_std']:.0f} "
                  f"vis={r['vis']} frame={r['frame']} off={r['center_off_px']}px")

    json.dump({"world": WORLD, "results": results, "uncovered": uncovered},
              open(os.path.join(OUT, "occupancy_results.json"), "w"), indent=1)

    # ---- debug overlays: frames containing errors (or first 3 if none)
    dbg_dir = os.path.join(OUT, "debug")
    os.makedirs(dbg_dir, exist_ok=True)
    dbg_frames = sorted({r["frame"] for r in wrong.values()} or
                        {r["frame"] for r in list(results.values())[:3]})
    by_frame = collections.defaultdict(list)
    for k, r in results.items():
        by_frame[r["frame"]].append((k, r))
    for fi in dbg_frames:
        img = Image.open(os.path.join(OUT, f"frame_{fi:03d}.png")).convert("RGB")
        img = img.resize((IMG_W * 3, IMG_H * 3), Image.NEAREST)
        d = ImageDraw.Draw(img)
        pose = next(p for p in poses if p["i"] == fi)
        for b in bays:
            ring_px = [project(x, y, pose) for x, y in b["ring"]]
            if not all(-20 <= u < IMG_W + 20 and -20 <= v < IMG_H + 20
                       for u, v in ring_px):
                continue
            r = results.get(b["id"])
            col = ("yellow" if r is None or r["frame"] != fi else
                   "red" if r["pred"] != r["gt"] else
                   "lime" if not r["pred"] else "deepskyblue")
            d.polygon([(u * 3, v * 3) for u, v in ring_px], outline=col, width=2)
        img.save(os.path.join(dbg_dir, f"debug_{fi:03d}.png"))
    print(f"debug overlays -> {dbg_dir} (frames {dbg_frames})")


if __name__ == "__main__":
    main()
