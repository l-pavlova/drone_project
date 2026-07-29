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
project = so.project
bay_features = so.bay_features
classify = so.classify
to_enu = so.to_enu


def score_frame(img_arr, bays, pose):
    """Classify every bay sufficiently visible in one nadir frame.

    Args:
        img_arr: HxWx3 RGB numpy array of the frame.
        bays: iterable of {"id": str, "ring": [(x, y), ...]} in ENU metres.
        pose: dict with x, y, alt, yaw (a poses.json record).

    Returns a list of per-bay single-view results:
        {"bay_id", "occupied", "off", "feat"}
    mirroring exactly the candidate-view + usability logic of the batch script.
    """
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
