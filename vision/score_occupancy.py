#!/usr/bin/env python3
"""
PARKDRONE vision stage v1: per-bay occupancy from captured nadir frames.

Georeferencing-first approach: bay polygons (data/block_bays.geojson) and the
drone pose per frame (sim/output/<survey_area>/poses.json) live in the same pinned
ENU frame as the world, so each bay can be PROJECTED into each frame. For
every bay we pick the best view (bay fully inside the frame, closest to the
image centre), classify the bay crop occupied/free, and score the predictions
against sim/worlds/<survey_area>.ground_truth.json.

    python score_occupancy.py [survey_area]      # default fmi_block_4st

Writes sim/output/<survey_area>/occupancy_results.json and annotated debug frames to
sim/output/<survey_area>/debug/. The v1.5 classifier is deliberately simple (painted
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

SURVEY_AREA = sys.argv[1] if len(sys.argv) > 1 else "fmi_block_4st"
OUT = os.path.join(ROOT, "sim", "output", SURVEY_AREA)
GT_FILE = os.path.join(ROOT, "sim", "worlds",
                       "ground_truth.json" if SURVEY_AREA == "fmi_block"
                       else f"{SURVEY_AREA}.ground_truth.json")
BAYS = os.path.join(ROOT, "data", "block_bays.geojson")

# camera intrinsics of the Mavic2Pro proto camera (see parkdrone.py startup log)
IMG_W, IMG_H = 400, 240
FOV = 0.7854                     # horizontal, radians
EDGE_MARGIN = 2                  # px: ignore pixels this close to the frame edge
INNER = 0.78                     # sample only the inner fraction of the bay
                                 # (excludes the painted outline + neighbours,
                                 # but keeps enough of a car's dark glass/wheels)
                                 # Reporting/debug only since CORE_W/CORE_LEN
                                 # took over the classified region.
CORE_LEN = 0.55                  # central fraction of the bay's LONG axis used
                                 # for classification (immune to overhang from
                                 # the adjacent bays, which enters at the ends)
CORE_W = 0.70                    # central fraction of the bay's SHORT axis.
                                 # This is the clearance from the bay's OWN
                                 # painted outline, and it is the binding one:
                                 # on a 2.2 m bay the side lines run 0.98-1.10 m
                                 # off centre, so INNER=0.78 left only 0.12 m
                                 # (<2 px at 30 m) while the measured per-frame
                                 # projection error reaches ~0.6 m. Paint then
                                 # leaks into the crop and lifts chroma /
                                 # brightness / std on a FREE bay towards the
                                 # occupied side. 0.70 gives 0.21 m.
                                 # CHOSEN ON THE CALIBRATION WORLD ONLY: 4st
                                 # holds 100% from 0.78 down to 0.66 and starts
                                 # missing real cars at 0.62, so 0.70 keeps two
                                 # steps of margin from that cliff. Do NOT pick
                                 # this by what scores best on fmi_block - that
                                 # is the held-out world, and its curve is
                                 # non-monotonic (95.2/95.2/97.6/95.2/95.2 at
                                 # 0.78/0.74/0.70/0.66/0.62) because bay 17686
                                 # sits right on the chroma threshold and flips.
                                 # Shrinking the two axes SEPARATELY matters:
                                 # the old uniform INNER shrink also ate the
                                 # length, which CORE_LEN already guards.
MIN_VIS = 0.70                   # a view is usable if at least this fraction of
                                 # the core area is inside the frame

# MUST match ORIGIN in sim/generate_world.py (the project's shared ENU frame)
ORIGIN = (42.6747105, 23.3298956)
MLAT = 111320.0
MLON = 111320.0 * math.cos(math.radians(ORIGIN[0]))


def to_enu(lon, lat):
    return (lon - ORIGIN[1]) * MLON, (lat - ORIGIN[0]) * MLAT


def pose_idx(pose):
    """A pose record's frame index (which frame_###.png it belongs to).

    Canonical key is `frame_idx`, matching frame.frame_idx and
    observation.frame_idx. `i` is the legacy name parkdrone.py wrote before
    2026-08-01; accepting it keeps every poses.json already on disk replayable
    (including a long patrol interrupted mid-resume) and keeps older drone
    builds ingestable.
    """
    return pose["frame_idx"] if "frame_idx" in pose else pose["i"]


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
    """Bays that exist in this survey area's ground truth, as ENU polygons."""
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


def _shrink_axis(ring_px, frac, along_long):
    """Shrink a 4-corner ring along ONE axis, leaving the other full.

    Each corner slides toward its neighbour across the chosen axis, so the ring
    stays a rectangle of the same orientation.
    """
    t = (1.0 - frac) / 2.0
    first_is_long = math.dist(ring_px[0], ring_px[1]) >= math.dist(ring_px[1], ring_px[2])
    if first_is_long == along_long:
        pairs = [(0, 1), (1, 0), (2, 3), (3, 2)]    # slide along edges 0-1 / 2-3
    else:
        pairs = [(0, 3), (1, 2), (2, 1), (3, 0)]    # slide along edges 1-2 / 3-0
    return [(ring_px[a][0] + (ring_px[b][0] - ring_px[a][0]) * t,
             ring_px[a][1] + (ring_px[b][1] - ring_px[a][1]) * t)
            for a, b in pairs]


def shrink_long(ring_px, frac):
    """Shrink a 4-corner ring along its LONG axis only (keeps full width)."""
    return _shrink_axis(ring_px, frac, True)


def shrink_short(ring_px, frac):
    """Shrink a 4-corner ring across its SHORT axis only (keeps full length)."""
    return _shrink_axis(ring_px, frac, False)


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
    # bright near-achromatic pixels: white/silver car bodywork (or the painted
    # outline, which the INNER shrink keeps out of the sampled region)
    paint = (mn > 140) & ((mx - mn) < 45)
    # much darker than the bay asphalt: glass, tyres, the shadow under a car
    dark = (px.mean(axis=1) < 55)
    return {"paint_frac": float(paint.mean()),
            "dark_frac": float(dark.mean()),
            "chroma": float((mx - mn).mean()),      # coloured bodywork pops
            "brightness": float(px.mean()),
            "std": float(px.std())}, vis


def bay_features(img_arr, ring_px):
    """Stats over the bay's inner region and its core. Returns None unless
    enough of the CORE is visible in this frame to classify from.

    The core is shrunk on each axis for a different reason, so the two fractions
    are separate: CORE_W across the width buys clearance from the bay's own
    painted outline, CORE_LEN along the length keeps a neighbour's overhang out.
    """
    inner_poly = shrink(ring_px, INNER)                # reporting/debug only
    core_poly = shrink_long(shrink_short(ring_px, CORE_W), CORE_LEN)
    core, core_vis = region_stats(img_arr, core_poly)
    if core is None or core_vis < MIN_VIS:
        return None
    feat = {"core_" + k: v for k, v in core.items()}
    inner, _ = region_stats(img_arr, inner_poly)    # reporting/debug only
    feat.update(inner or {})
    feat["vis"] = round(core_vis, 2)
    return feat


# calibrated on fmi_block_4st against ground truth (v2 heuristic for the
# realistic outline-marked world; a learned classifier can replace this
# without touching projection/view selection/scoring). Free-core envelope on
# the dev world: chroma <= 12.0, dark 0, paint <= 0.10, brightness 84.7-99.8,
# std <= 45.5; every occupied view clears T_CHROMA alone (min 13.5).
T_CHROMA = 12.7                  # asphalt is near-achromatic; any bodywork isn't
T_DARK = 0.02                    # glass/tyres/under-car shadow, well below asphalt
T_PAINT = 0.15                   # white/silver roof (the outline stays excluded)
T_BRIGHT_LO, T_BRIGHT_HI = 82, 102   # asphalt tone envelope
T_STD = 50                       # texture: panel gaps, windscreen edges


def classify(feat):
    """v2 heuristic on the bay CORE: a free core is uniform asphalt; a car
    differs from it in chroma (coloured bodywork), brightness (light or very
    dark bodywork), darkness (glass/shadow) or texture. Ends of the bay are
    excluded so a neighbour's overhang cannot fake occupancy."""
    return (feat["core_chroma"] > T_CHROMA
            or feat["core_dark_frac"] > T_DARK
            or feat["core_paint_frac"] > T_PAINT
            or not T_BRIGHT_LO < feat["core_brightness"] < T_BRIGHT_HI
            or feat["core_std"] > T_STD)


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
            feat = bay_features(frame_arr(pose_idx(pose)), ring_px)
            if feat is not None:
                votes.append((feat["vis"], off, classify(feat), pose_idx(pose), feat))
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
    print(f"survey area {SURVEY_AREA}: {len(bays)} bays, {n} classified, {len(uncovered)} uncovered")
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

    json.dump({"survey_area": SURVEY_AREA, "results": results, "uncovered": uncovered},
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
        pose = next(p for p in poses if pose_idx(p) == fi)
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
