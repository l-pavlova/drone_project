"""Score one frame end to end: classify -> append observations -> recompute
bay_state -> return deltas. Shared by the in-process classify threads (jobs.py)
and the offline replay golden test (replay.py).

process_frame returns the deltas rather than publishing them itself; the caller
broadcasts them (the FastAPI hub in production, nothing in the replay test).
"""
from ..db import vision_db
from ..vision import ground_truth
from ..vision.scoring import score_frame


def process_frame(conn, survey_area, frame_idx, img_arr, pose, bays, frame_id=None,
                  gt=None, index=None):
    # Labels are a property of the survey area, not of the caller, so they are
    # resolved here — that way live ingest and the offline replay both record
    # them and accuracy is measured on the same path production runs. Passing
    # `gt` explicitly overrides the lookup; production areas simply have none.
    if gt is None:
        gt = ground_truth.labels_for(survey_area)
    scores = score_frame(img_arr, bays, pose, index=index)
    vision_db.insert_observations(conn, survey_area, frame_idx, scores, gt=gt)
    touched = [s["bay_id"] for s in scores]
    deltas = vision_db.recompute_states(conn, survey_area, touched, frame_idx)
    conn.commit()
    return {"scored": len(scores), "deltas": deltas}
