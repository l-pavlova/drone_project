"""Turn detector boxes into per-bay occupancy verdicts.

    python vision/detect_occupancy.py <stills_dir> [--model W] [--conf 0.25]
                                      [--cam dji] [--limit N] [--json OUT]

This is the join between the two halves of the project. `classify()` answers
"is there a car in THIS bay?" by cropping the bay out of the frame and reading
its colour statistics -- five thresholds calibrated on Webots tones, which is
why it is meaningless on a real photograph (TODO #6 measured that it has no
form that survives even a change of sunlight, let alone a change of renderer).
A whole-frame detector answers a different question -- "where are the cars?" --
and this module converts the second answer into the first.

**The rule, unchanged from `detect_baseline.py` where it was first written and
measured:** unproject each detection's box centre to the ground plane, and a bay
is occupied on this frame if some detection landed within `ASSIGN_MAX_M` of its
centroid. A bay is 2.2 x 4.5-6 m, so 3 m is about one bay width; further than
that and the detection is evidence about a different bay, or about a car that is
not parked in one at all.

Three properties of that rule are deliberate and worth not "fixing":

  * **the box CENTRE, not the box.** A nadir car's box is the car; its centre is
    the car's middle, which is the thing that has to be inside a bay. Testing
    box-vs-bay overlap instead would let a long vehicle claim the two bays it
    overhangs.
  * **assignment is not exclusive.** One detection within 3 m of two bay
    centroids marks both. That is the honest reading where the bay data itself
    has 222 overlapping pairs (TODO #1) -- an exclusive assignment would invent
    a precision the geometry does not have.
  * **only bays FULLY in shot vote.** The same rule the crops use, so a
    detector verdict and a heuristic verdict are counted on the same views and
    the two numbers are comparable. A bay half out of frame has half a chance of
    containing a detection.

Voting across frames is the caller's job (`vote()` here, `recompute_states` in
the web tier) because the two have different notions of which frames are in the
window -- but both are a strict majority over the views, and that must stay
true or a real number stops being comparable with a sim one.
"""
import argparse
import collections
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                  # noqa: E402
import check_nadir                                              # noqa: E402
import curb_runs as cr                                          # noqa: E402
import score_occupancy as so                                    # noqa: E402

ASSIGN_MAX_M = 3.0      # how far a detection may land from a bay's centre and
#   still count as that bay's car (see the module docstring)


def bay_votes_from_dets(dets, bays, pose, cam=None, assign_max_m=ASSIGN_MAX_M,
                        unassigned=None):
    """Detections on one frame -> a verdict for every bay fully in that frame.

    `dets` is [(box_xyxy, conf), ...] in that frame's pixel coordinates; `bays`
    is score_occupancy's dict shape (`id`, `ring`, `cx`, `cy`). Returns
    [{"bay_id", "occupied", "off", "det_score"}] -- the same contract
    `vision/scoring.py::score_frame` returns, so the web pipeline can take
    either backend without knowing which it has.

    **Pass a list as `unassigned` to be told about the cars this rule THROWS
    AWAY.** The loop runs over bays, so a detection matching no bay simply
    vanished, and the report that followed was a confident half-truth: flight
    0075 came out as "9 bays, all free" when what actually happened was "9 bays
    free, and 91 cars we saw and could not place". Zero occupied bays is a quiet
    signal and 91 unplaced cars is a loud one, so the omission hid exactly the
    thing worth noticing (2026-08-26). Each entry is
    {"x", "y", "conf", "nearest_bay", "nearest_m"} -- the distance is kept
    because it separates the two reasons a car goes unplaced: just outside the
    radius (geometry a few metres off) versus nowhere near anything (parking
    that is not in the dataset at all).

    It stays an opt-in out-parameter rather than a second return value because
    this function's return shape is a contract shared with the heuristic
    backend, and the web pipeline switches between them blind.

    `off` is the bay centroid's distance from the image centre in pixels, kept
    for the same reason the heuristic keeps it: it is how a marginal view is
    recognised after the fact. `det_score` is the confidence of the detection
    that claimed the bay, or None for a bay left free -- there is no such thing
    as a confidence for "no car was found here", and inventing one (1 - max
    conf, say) would put a number in the database that means nothing.
    """
    w, h, _ = so._intrinsics(cam)
    hits = []
    for box, conf in dets:
        g = so.unproject((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                         pose, cam=cam)
        if g:
            hits.append((g[0], g[1], conf))

    out = []
    claimed = set()
    for b in bays:
        ring = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
        us = [p[0] for p in ring]
        vs = [p[1] for p in ring]
        if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
            continue            # not fully in shot: the rule the crops use
        cx, cy = b["cx"], b["cy"]
        best = None
        for k, (hx, hy, conf) in enumerate(hits):
            if math.hypot(hx - cx, hy - cy) <= assign_max_m:
                claimed.add(k)
                if best is None or conf > best:
                    best = conf
        out.append({
            "bay_id": b["id"],
            "occupied": best is not None,
            "off": math.hypot(sum(us) / len(us) - w / 2.0,
                              sum(vs) / len(vs) - h / 2.0),
            "det_score": best,
        })

    if unassigned is not None:
        # Measured against EVERY bay, not just the ones fully in shot: a car may
        # sit in a bay the frame only half sees, and calling that "nowhere near
        # a bay" would overstate the gap this list exists to measure.
        for k, (hx, hy, conf) in enumerate(hits):
            if k in claimed:
                continue
            near_id, near_d = None, None
            for b in bays:
                d = math.hypot(hx - b["cx"], hy - b["cy"])
                if near_d is None or d < near_d:
                    near_id, near_d = b["id"], d
            unassigned.append({"x": hx, "y": hy, "conf": conf,
                               "nearest_bay": near_id, "nearest_m": near_d})
    return out


def ground_quad(box, pose, cam=None):
    """A detection box -> its four corners on the ground, in ENU metres.

    `bay_votes_from_dets` unprojects only the box CENTRE, which is right for the
    question it asks (is the car's middle inside this rectangle). The run layer
    asks a different one -- how much CURB does this car consume -- and that needs
    the extent, so all four corners go through `so.unproject`.

    Note what the quad is and is not. A YOLO box is axis-aligned in the IMAGE, so
    this quad is the box's footprint, not the car's: for a car lying at 45 degrees
    to the frame the box is nearly square and its long axis means nothing. That is
    why nothing here tries to recover a car heading from it -- `run_extent` projects
    the corners onto the RUN's tangent instead, which is the only direction the
    answer actually depends on, and which is tight in the common case because the
    drone flies along the street it is surveying.
    """
    x0, y0, x1, y1 = box[0], box[1], box[2], box[3]
    out = []
    for u, v in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        g = so.unproject(u, v, pose, cam=cam)
        if g is None:
            return None
        out.append((g[0], g[1]))
    return out


def run_extent(run, quad):
    """(s0, s1) -- the arclength stretch of `run` this ground quad covers."""
    ss = []
    for x, y in quad:
        got = cr.locate(run, x, y)
        if got is not None:
            ss.append(got[0])
    return (min(ss), max(ss)) if ss else None


def observed_spans(run, pose, cam=None, step=cr.CELL_M):
    """Which stretches of a run this frame actually LOOKED at -> [(s0, s1), ...].

    Sampled every `step` metres and tested by projecting back into the image, so a
    run that leaves and re-enters the frame yields several spans rather than one
    wrong one.

    **This is a GEOMETRIC mask and it overstates observation.** It says the curb
    was inside the frame, which is a weaker claim than the curb having been
    visible: flight 0074's `frame_0050.jpg` is a street essentially entirely under
    summer canopy, and every metre of it passes this test. Until a radiometric
    occlusion test exists, treat `observed_fraction` as an upper bound -- and never
    let it license reporting unobserved curb as free.
    """
    w, h, _ = so._intrinsics(cam)
    length = run["length_m"]
    line, s = run["line"], run["s"]
    spans, open_at = [], None
    n = max(2, int(length / step) + 1)
    for k in range(n):
        sk = length * k / (n - 1)
        # walk the polyline to the point at arclength sk
        i = 0
        while i < len(s) - 2 and s[i + 1] < sk:
            i += 1
        seg = (s[i + 1] - s[i]) or 1.0
        t = (sk - s[i]) / seg
        x = line[i][0] + t * (line[i + 1][0] - line[i][0])
        y = line[i][1] + t * (line[i + 1][1] - line[i][1])
        p = so.project(x, y, pose, cam=cam)
        inside = p is not None and 0 <= p[0] < w and 0 <= p[1] < h
        if inside and open_at is None:
            open_at = sk
        elif not inside and open_at is not None:
            spans.append((open_at, sk))
            open_at = None
    if open_at is not None:
        spans.append((open_at, length))
    return spans


def run_intervals_from_dets(dets, runs, pose, cam=None, lateral_max_m=6.0):
    """Per-run occupied intervals and observed spans for ONE frame.

    -> {run_id: {"occupied": [(s0, s1), ...], "observed": [(s0, s1), ...],
                 "dets": n}}

    Runs alongside `bay_votes_from_dets`, never instead of it: the per-bay verdict
    is still what the sim path, the golden fixtures and the web tier are scored on,
    and the two answers are meant to be reported side by side until the run layer
    has earned the swap against real ground truth.

    A detection is assigned to the NEAREST run within `lateral_max_m`, and that
    assignment IS exclusive -- unlike the per-bay rule, which is deliberately not.
    The reason the per-bay rule stays inclusive is that 222 bay pairs overlap, so
    an exclusive choice there would invent precision; two runs, by construction,
    are at least LATERAL_MAX apart across the street, so a car genuinely belongs to
    one of them and letting it occupy curb on both would double-count it.
    """
    out = {}
    for r in runs:
        out[r["run_id"]] = {"occupied": [], "observed": observed_spans(r, pose, cam),
                            "dets": 0}

    for box, conf in dets:
        quad = ground_quad(box, pose, cam=cam)
        if quad is None:
            continue
        cx = sum(p[0] for p in quad) / 4.0
        cy = sum(p[1] for p in quad) / 4.0
        best = None
        for r in runs:
            got = cr.locate(r, cx, cy)
            if got is None:
                continue
            _s, _lat, dist = got
            if dist > lateral_max_m:
                continue
            if best is None or dist < best[0]:
                best = (dist, r)
        if best is None:
            continue
        r = best[1]
        span = run_extent(r, quad)
        # A span of zero length means every corner clamped to the same endpoint,
        # i.e. the car is off the end of the run rather than on it. The distance
        # gate above should already have caught that; this is the backstop, since
        # such an interval is invisible in the totals (it adds no occupied length)
        # while still counting as a detection seen.
        if span is None or span[1] - span[0] < 1e-3:
            continue
        rec = out[r["run_id"]]
        rec["occupied"].append((max(0.0, span[0]), min(r["length_m"], span[1])))
        rec["dets"] += 1
    return out


CLUSTER_M = 2.0      # single-link radius for merging views of one car
MIN_VIEWS = 2        # frame-distinct detections before a cluster counts as a car


def cluster_instances(frames, cam=None, cluster_m=CLUSTER_M,
                      min_views=MIN_VIEWS):
    """Detections across frames -> distinct parked cars, as ground points.

    Unprojects each box centre and hands the points to `curb_runs.cluster_points`,
    which owns the clustering so the offline scorer and the web recompute cannot
    disagree about what counts as one car.
    """
    pts = []
    for i, (pose, dets) in enumerate(frames):
        for box, conf in dets:
            g = so.unproject((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                             pose, cam=cam)
            if g:
                pts.append((g[0], g[1], i))
    return cr.cluster_points(pts, cluster_m=cluster_m, min_views=min_views)


def vote(per_frame):
    """[[score, ...], ...] over frames -> {bay_id: (occupied, views, n_occ)}.

    Strict majority, identical to `score_occupancy.main()` and to the web
    tier's `_RECOMPUTE_SQL`. A tie reads FREE, which is the conservative answer
    for a parking map: telling a driver a space is taken when it is not costs
    them nothing, telling them one is free when it is not sends them there.
    """
    tally = {}
    for scores in per_frame:
        for s in scores:
            occ, n = tally.get(s["bay_id"], (0, 0))
            tally[s["bay_id"]] = (occ + (1 if s["occupied"] else 0), n + 1)
    return {bay: (occ * 2 > n, n, occ) for bay, (occ, n) in tally.items()}


# --------------------------------------------------------------------- CLI

def run(stills, model_name, conf, cam, limit=0, every=1, imgsz=1024,
        screen_nadir=True):
    """Score a real stills directory end to end, on disk, with no server."""
    import numpy as np
    from PIL import Image
    from ultralytics import YOLO

    poses = json.load(open(os.path.join(stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p][::every]
    if limit:
        poses = poses[:limit]
    bays = so.load_all_bays()
    model = YOLO(model_name)
    print(f"{len(poses)} poses, {len(bays)} bays, {cam!r}\n"
          f"model {model_name}, conf {conf}, imgsz {imgsz}")

    per_frame = []
    kept = []
    n_det = 0
    skipped = []
    unassigned = []
    for pose in poses:
        img = Image.open(os.path.join(stills, pose["file"])).convert("RGB")
        arr = np.array(img)
        if screen_nadir and check_nadir.is_oblique(arr[:, :, ::-1])[2]:
            # An oblique frame carries no usable projection: `project()` assumes
            # the optical axis points down, and a frame aimed at the horizon puts
            # its cars tens of metres from where they are -- silently, and with
            # full confidence. Dropping it loses the frame's evidence; keeping it
            # poisons other bays with it.
            skipped.append(pose["file"])
            continue
        # ultralytics reads a numpy array as BGR (the cv2 convention it trains
        # with); `arr` is RGB from PIL, so hand the detector a reversed view or
        # it sees every image with red and blue swapped. Measured on flight
        # 0074's 108 nadir frames: 108 detections swapped against 173 correct.
        res = model.predict(arr[:, :, ::-1], conf=conf, imgsz=imgsz,
                            verbose=False)[0]
        dets = list(zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist()))
        n_det += len(dets)
        per_frame.append(bay_votes_from_dets(dets, bays, pose, cam=cam,
                                             unassigned=unassigned))
        kept.append(pose)
    if skipped:
        print(f"  {len(skipped)} frame(s) skipped as non-nadir: "
              f"{skipped[0]} .. {skipped[-1]}")
    return kept, bays, per_frame, n_det, unassigned


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--model", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "runs", "merged1", "weights", "best.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--every", type=int, default=1)
    # Must match the weights' training size (merged1: 1024) or a 385 px car
    # arrives letterboxed down to 64 px and the model quietly under-detects.
    ap.add_argument("--imgsz", type=int, default=1024)
    ap.add_argument("--json", default=None)
    ap.add_argument("--no-screen", action="store_true",
                    help="do NOT drop frames the nadir screen flags as oblique "
                         "(vision/check_nadir.py). Only for inspecting what such "
                         "a frame would have produced -- its projection is wrong.")
    a = ap.parse_args()

    sys.stdout.reconfigure(encoding="utf-8")
    cam = cameras.get(a.cam)
    poses, bays, per_frame, n_det, unassigned = run(
        a.stills, a.model, a.conf, cam, a.limit, a.every, a.imgsz,
        screen_nadir=not a.no_screen)
    result = vote(per_frame)
    occ = sum(1 for v in result.values() if v[0])
    views = [v[1] for v in result.values()]
    print(f"\n{n_det} detections over {len(poses)} frames "
          f"({n_det / max(len(poses), 1):.1f} per frame)")
    print(f"{len(result)} bays voted of {len(bays)} in the dataset: "
          f"{occ} occupied, {len(result) - occ} free")
    if views:
        views.sort()
        print(f"views per bay: median {views[len(views) // 2]}, "
              f"min {views[0]}, max {views[-1]}")
        single = sum(1 for v in views if v == 1)
        print(f"  {single} bay(s) decided by a SINGLE view -- the multi-view "
              f"majority is the defence against one bad look, and those bays "
              f"do not have it")

    # The cars this survey saw and could NOT place. Printed next to the bay
    # counts and never folded into them: "9 bays, all free" is a true statement
    # that becomes a misleading one the moment 91 detections have been dropped
    # in silence to produce it.
    print(f"\n{len(unassigned)} detection(s) of {n_det} matched NO bay "
          f"({100 * len(unassigned) / max(n_det, 1):.0f}%)")
    if unassigned:
        ds = sorted(u["nearest_m"] for u in unassigned)
        n = len(ds)
        just_out = sum(1 for d in ds if d <= 2 * ASSIGN_MAX_M)
        print(f"  distance to the nearest bay: median {ds[n // 2]:.2f} m, "
              f"p10 {ds[n // 10]:.2f}, p90 {ds[9 * n // 10]:.2f}")
        print(f"  {just_out} within {2 * ASSIGN_MAX_M:.0f} m -- a few metres of "
              f"geometry would place these; the rest are parked where the "
              f"dataset maps no bay at all")

    if a.json:
        with open(a.json, "w", encoding="utf-8", newline="") as fh:
            json.dump({"stills": a.stills, "model": a.model, "conf": a.conf,
                       "detections": n_det,
                       "unassigned": [
                           {"x": round(u["x"], 2), "y": round(u["y"], 2),
                            "conf": round(u["conf"], 3),
                            "nearest_bay": u["nearest_bay"],
                            "nearest_m": round(u["nearest_m"], 2)}
                           for u in unassigned],
                       "bays": {b: {"occupied": v[0], "views": v[1],
                                    "votes_occupied": v[2]}
                                for b, v in sorted(result.items())}}, fh, indent=1)
        print(f"wrote {a.json}")


if __name__ == "__main__":
    main()
