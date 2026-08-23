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
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                  # noqa: E402
import score_occupancy as so                                    # noqa: E402

ASSIGN_MAX_M = 3.0      # how far a detection may land from a bay's centre and
#   still count as that bay's car (see the module docstring)


def bay_votes_from_dets(dets, bays, pose, cam=None, assign_max_m=ASSIGN_MAX_M):
    """Detections on one frame -> a verdict for every bay fully in that frame.

    `dets` is [(box_xyxy, conf), ...] in that frame's pixel coordinates; `bays`
    is score_occupancy's dict shape (`id`, `ring`, `cx`, `cy`). Returns
    [{"bay_id", "occupied", "off", "det_score"}] -- the same contract
    `vision/scoring.py::score_frame` returns, so the web pipeline can take
    either backend without knowing which it has.

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
    for b in bays:
        ring = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
        us = [p[0] for p in ring]
        vs = [p[1] for p in ring]
        if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
            continue            # not fully in shot: the rule the crops use
        cx, cy = b["cx"], b["cy"]
        best = None
        for hx, hy, conf in hits:
            if math.hypot(hx - cx, hy - cy) <= assign_max_m:
                if best is None or conf > best:
                    best = conf
        out.append({
            "bay_id": b["id"],
            "occupied": best is not None,
            "off": math.hypot(sum(us) / len(us) - w / 2.0,
                              sum(vs) / len(vs) - h / 2.0),
            "det_score": best,
        })
    return out


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

def run(stills, model_name, conf, cam, limit=0, every=1, imgsz=1024):
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
    n_det = 0
    for pose in poses:
        img = Image.open(os.path.join(stills, pose["file"])).convert("RGB")
        res = model.predict(np.array(img), conf=conf, imgsz=imgsz, verbose=False)[0]
        dets = list(zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist()))
        n_det += len(dets)
        per_frame.append(bay_votes_from_dets(dets, bays, pose, cam=cam))
    return poses, bays, per_frame, n_det


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
    a = ap.parse_args()

    sys.stdout.reconfigure(encoding="utf-8")
    cam = cameras.get(a.cam)
    poses, bays, per_frame, n_det = run(a.stills, a.model, a.conf, cam,
                                        a.limit, a.every, a.imgsz)
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

    if a.json:
        with open(a.json, "w", encoding="utf-8", newline="") as fh:
            json.dump({"stills": a.stills, "model": a.model, "conf": a.conf,
                       "bays": {b: {"occupied": v[0], "views": v[1],
                                    "votes_occupied": v[2]}
                                for b, v in sorted(result.items())}}, fh, indent=1)
        print(f"wrote {a.json}")


if __name__ == "__main__":
    main()
