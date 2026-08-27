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
from ..db import run_db, vision_db, web_db
from ..vision import ground_truth
from ..vision import runs as run_layer
from ..vision.scoring import ASSIGN_MAX_M, score_frame, score_frame_detector
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
    # Cars the frame saw and could not attribute to any bay (migration 0013).
    # Only the detector can have them; the heuristic reads bay crops, so there is
    # nothing for it to fail to place, and its count stays NULL rather than 0.
    unassigned = None
    raw_dets = None
    if backend_name() == "detector":
        unassigned = []
        raw_dets = [] if config.RUN_LAYER else None
        scores = score_frame_detector(img_arr, bays, pose, index=index,
                                      model=model, cam=cam,
                                      conf=config.DETECTOR_CONF,
                                      imgsz=config.DETECTOR_IMGSZ,
                                      unassigned=unassigned,
                                      dets_out=raw_dets)
    else:
        scores = score_frame(img_arr, bays, pose, index=index)
    vision_db.insert_observations(conn, survey_area, frame_idx, scores, gt=gt,
                                  mission_id=mission_id,
                                  backend=backend_name(),
                                  # (0011) makes the append idempotent, so a
                                  # frame re-claimed after a crash records one
                                  # look rather than double-voting
                                  frame_id=frame_id)
    if unassigned is not None and frame_id is not None:
        near = sum(1 for u in unassigned
                   if u["nearest_m"] is not None
                   and u["nearest_m"] <= 2 * ASSIGN_MAX_M)
        web_db.record_unassigned(conn, frame_id, len(unassigned), near)
    # The curb-run layer, beside the per-bay one and sharing this frame's single
    # detector pass (migration 0014). It writes its own tables and recomputes its
    # own state, so nothing above this point changes and the bay answer is
    # bit-identical with the layer on or off.
    run_result = None
    if raw_dets is not None:
        r_dets, r_spans = run_layer.frame_evidence(raw_dets, pose, cam=cam)
        if r_spans:
            r_touched = run_db.record_frame_runs(conn, survey_area, frame_idx,
                                                 frame_id, mission_id,
                                                 r_dets, r_spans)
            run_result = run_db.recompute_run_states(conn, survey_area, r_touched,
                                                     backend=backend_name())

    touched = [s["bay_id"] for s in scores]
    deltas = vision_db.recompute_states(conn, survey_area, touched, frame_idx)
    if publish:
        delta_channel.publish(conn, deltas)
    conn.commit()
    return {"scored": len(scores), "deltas": deltas,
            "runs": len(run_result) if run_result else 0}
