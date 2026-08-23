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

import cameras  # noqa: E402
import score_occupancy as so  # noqa: E402
from detect_occupancy import bay_votes_from_dets  # noqa: E402

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


# The camera is not exactly nadir (see score_occupancy.camera_axes), so the
# footprint is not centred on the drone. Measured worst-case displacement over a
# real flight is ~0.6 m at 30 m; a tilt of this many radians covers it with room
# to spare, and the reach is only a conservative rejection radius anyway.
TILT_ALLOW = 0.06


def footprint_reach(alt, cam=None):
    """Half-diagonal, in metres, of the ground rectangle a frame covers.

    Inverts `project`'s scale: it maps ground metres to pixels with
    k = IMG_W / (2*alt*tan(FOV/2)), so the visible half-width is alt*tan(FOV/2)
    and the half-height is that times the aspect ratio. At 30 m: 12.4 x 7.5 m,
    i.e. the ~25x15 m footprint the patrol spacing is designed around.

    A point farther than this from the drone cannot be inside the frame, which is
    what makes it a safe rejection radius -- plus an allowance for the camera
    tilt, which slides the footprint off the drone's own position and would
    otherwise let this cheap test reject a bay that is genuinely in shot.
    """
    fov = FOV if cam is None else cam.fov_h
    aspect = (IMG_H / IMG_W) if cam is None else (cam.h / cam.w)
    half_w = alt * math.tan(fov / 2.0)
    half_h = half_w * aspect
    return math.hypot(half_w, half_h) + alt * math.tan(TILT_ALLOW)


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


# --------------------------------------------------------------------------
# The learned-detector backend.
#
# It answers the SAME question as score_frame -- which bays in this frame hold
# a car -- and returns the identical contract, so processing/pipeline.py picks
# between them by configuration and nothing downstream knows the difference.
#
# The two are NOT interchangeable in practice and must not be treated as such:
# `classify()` is five thresholds calibrated on Webots tones and is the only
# thing that reproduces the sim's numbers on record (replay fmi_block 42/42 at
# 100%), while COCO-scale detectors find literally 0 of 52 cars in those same
# rendered frames. The detector is for real photographs, where the heuristic is
# meaningless. Which is why the default backend is the heuristic and the
# detector is opted into.


def load_detector(weights):
    """One YOLO instance. Call once PER THREAD -- see score_frame_detector."""
    from ultralytics import YOLO
    return YOLO(weights)


def score_frame_detector(img_arr, bays, pose, index=None, model=None,
                         cam=None, conf=0.25, imgsz=1024):
    """Detector counterpart of score_frame, returning the same contract.

    Runs the detector ONCE on the whole frame, then hands the boxes to the
    shared `bay_votes_from_dets` rule (vision/detect_occupancy.py) that the
    offline sim baseline also uses -- so a real verdict and a sim verdict are
    produced by the same geometry, and only the box source differs.

    `model` must be a per-thread instance: ultralytics keeps mutable predictor
    state on the model object, so sharing one across the classify threads is a
    data race. A yolov8s is ~22 MB, so a copy per thread is cheaper than the
    lock that would be needed to share one.

    `feat` carries only `det_score` -- the heuristic's five colour statistics do
    not exist here and are left ABSENT rather than zero-filled, because a zero
    in `core_chroma` would be indistinguishable from a measured zero when those
    columns are read back for analysis.
    """
    import numpy as np

    if model is None:
        raise ValueError("score_frame_detector needs a per-thread model; "
                         "call load_detector() at worker start")
    if index is not None:
        bays = index.visible(bays, pose["x"], pose["y"],
                             footprint_reach(pose["alt"], cam))
    res = model.predict(np.asarray(img_arr), conf=conf, imgsz=imgsz,
                        verbose=False)[0]
    dets = list(zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist()))
    out = bay_votes_from_dets(dets, bays, pose, cam=cam)
    for s in out:
        s["off"] = round(s["off"], 1)
        score = s.pop("det_score")
        s["feat"] = {} if score is None else {"det_score": round(score, 4)}
    return out
