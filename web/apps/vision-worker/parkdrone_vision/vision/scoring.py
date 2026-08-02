"""Bridge to the calibrated offline classifier + a single-frame scorer.

We import the project's vision stage so projection, view-selection and the tuned
`classify()` thresholds stay in ONE place. `vision/score_occupancy.py` guards its
batch `main()` under `if __name__ == "__main__"`, so importing it is side-effect
free (module-level code only computes paths, no I/O).
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
# parkdrone_vision/vision -> parkdrone_vision -> vision-worker -> apps -> web -> <repo root> -> vision
_VISION_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "..", "..", "..", "vision"))
if _VISION_DIR not in sys.path:
    sys.path.insert(0, _VISION_DIR)

import score_occupancy as so  # noqa: E402

# Re-export the calibrated pieces so the rest of the worker never re-implements them.
IMG_W, IMG_H = so.IMG_W, so.IMG_H
FOV = so.FOV
project = so.project
bay_features = so.bay_features
classify = so.classify
to_enu = so.to_enu
# Reading a pose's frame index is format knowledge, not classifier logic, but it
# lives with the format (score_occupancy owns poses.json) and reaches the web
# tier by the same one-place-only route as the rest.
pose_idx = so.pose_idx


def footprint_reach(alt):
    """Half-diagonal, in metres, of the ground rectangle a nadir frame covers.

    Inverts `project`'s scale: it maps ground metres to pixels with
    k = IMG_W / (2*alt*tan(FOV/2)), so the visible half-width is alt*tan(FOV/2)
    and the half-height is that times the aspect ratio. At 30 m: 12.4 x 7.5 m,
    i.e. the ~25x15 m footprint the patrol spacing is designed around.

    A point farther than this from the drone cannot be inside the frame, which is
    what makes it a safe rejection radius.
    """
    half_w = alt * math.tan(FOV / 2.0)
    half_h = half_w * (IMG_H / IMG_W)
    return math.hypot(half_w, half_h)


def score_frame(img_arr, bays, pose, index=None):
    """Classify every bay sufficiently visible in one nadir frame.

    Args:
        img_arr: HxWx3 RGB numpy array of the frame.
        bays: iterable of {"id": str, "ring": [(x, y), ...]} in ENU metres.
        pose: dict with x, y, alt, yaw (a poses.json record).
        index: optional BayIndex over `bays`. Given one, bays outside the camera
            footprint are rejected by a vectorised distance test instead of being
            projected vertex-by-vertex. Results are identical either way — the
            test is conservative — but the per-frame cost stops scaling with the
            size of the bay dataset, which is what keeps this affordable beyond
            one city block.

    Returns a list of per-bay single-view results:
        {"bay_id", "occupied", "off", "feat"}
    mirroring exactly the candidate-view + usability logic of the batch script.
    """
    if index is not None:
        bays = index.visible(bays, pose["x"], pose["y"], footprint_reach(pose["alt"]))
    out = []
    for b in bays:
        ring_px = [project(x, y, pose) for x, y in b["ring"]]
        us = [p[0] for p in ring_px]
        vs = [p[1] for p in ring_px]
        # bay bbox must overlap the frame at all
        if max(us) < 0 or min(us) >= IMG_W or max(vs) < 0 or min(vs) >= IMG_H:
            continue
        feat = bay_features(img_arr, ring_px)
        if feat is None:  # not enough of the core visible to classify
            continue
        off = math.hypot(sum(us) / 4 - IMG_W / 2, sum(vs) / 4 - IMG_H / 2)
        out.append(
            {
                "bay_id": b["id"],
                "occupied": bool(classify(feat)),
                "off": round(off, 1),
                "feat": feat,
            }
        )
    return out
