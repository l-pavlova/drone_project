"""Score one frame end to end: classify -> append observations -> recompute
bay_state -> return deltas. Shared by the in-process classify threads (jobs.py)
and the offline replay golden test (replay.py).

process_frame PUBLISHES the deltas here, inside the same transaction that wrote
them, and returns them for the caller's counters. Publishing means a bay_delta
row + a pg_notify; every replica's listener thread then pushes to its own
clients. Doing it in-transaction is what makes the wire and the database agree:
NOTIFY fires on COMMIT, so nothing is ever announced for state that rolled back.
`publish=False` opts the offline replay golden test out — it has no server, no
listener and nobody to tell.

**Two backends answer the same question here** (see config.OCCUPANCY_BACKEND):
the calibrated per-bay heuristic, which is the only thing that reproduces the
simulator's numbers, and a fine-tuned whole-frame detector, which is the only
thing that works on real photographs. They return the identical contract, so
everything downstream of `scores` -- observations, the vote, bay_state, the
WebSocket delta, the map -- is shared and neither backend gets its own path.
"""
from .. import config
from ..db import vision_db
from ..vision import ground_truth
from ..vision.scoring import score_frame, score_frame_detector
from . import deltas as delta_channel


def backend_name():
    """The configured backend, recorded on every observation as provenance.

    Once two models can decide a bay, "which one did?" stops being a deployment
    detail and becomes a property of the datum -- it is what lets a real result
    and a simulated one be told apart after the fact, and the two must never be
    averaged.
    """
    return "detector" if config.OCCUPANCY_BACKEND == "detector" else "heuristic"


def process_frame(conn, survey_area, frame_idx, img_arr, pose, bays, frame_id=None,
                  gt=None, index=None, mission_id=None, model=None, cam=None,
                  publish=True):
    # Labels are a property of the survey area, not of the caller, so they are
    # resolved here — that way live ingest and the offline replay both record
    # them and accuracy is measured on the same path production runs. Passing
    # `gt` explicitly overrides the lookup; production areas simply have none.
    if gt is None:
        gt = ground_truth.labels_for(survey_area)
    if backend_name() == "detector":
        scores = score_frame_detector(img_arr, bays, pose, index=index,
                                      model=model, cam=cam,
                                      conf=config.DETECTOR_CONF,
                                      imgsz=config.DETECTOR_IMGSZ)
    else:
        scores = score_frame(img_arr, bays, pose, index=index)
    vision_db.insert_observations(conn, survey_area, frame_idx, scores, gt=gt,
                                  mission_id=mission_id,
                                  backend=backend_name(),
                                  # (0011) makes the append idempotent, so a
                                  # frame re-claimed after a crash records one
                                  # look rather than double-voting
                                  frame_id=frame_id)
    touched = [s["bay_id"] for s in scores]
    deltas = vision_db.recompute_states(conn, survey_area, touched, frame_idx)
    if publish:
        delta_channel.publish(conn, deltas)
    conn.commit()
    return {"scored": len(scores), "deltas": deltas}
