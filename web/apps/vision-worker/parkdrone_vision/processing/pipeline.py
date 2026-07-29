"""Score one frame end to end: classify -> append observations -> recompute
bay_state -> return deltas. Shared by the in-process classify threads (jobs.py)
and the offline replay golden test (replay.py).

Delta fan-out is no longer Redis pub/sub: process_frame just returns the deltas
and the caller broadcasts them (the FastAPI hub in production, nothing in the
replay test).
"""
from ..db import vision_db
from ..vision.scoring import score_frame


def process_frame(conn, world, frame_idx, img_arr, pose, bays, frame_id=None, gt=None):
    scores = score_frame(img_arr, bays, pose)
    vision_db.insert_observations(conn, world, frame_idx, scores, gt=gt)
    touched = [s["bay_id"] for s in scores]
    deltas = vision_db.recompute_states(conn, world, touched, frame_idx)
    conn.commit()
    return {"scored": len(scores), "deltas": deltas}
