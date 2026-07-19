"""Score one frame end to end: classify -> append observations -> recompute
bay_state -> publish deltas. Shared by the live worker and the replay driver.
"""
import json

from . import db
from .config import DELTAS_CHANNEL
from .vision_core import score_frame


def process_frame(
    conn,
    world,
    frame_idx,
    img_arr,
    pose,
    bays,
    redis_client=None,
    frame_id=None,
    gt=None,
):
    scores = score_frame(img_arr, bays, pose)
    db.insert_observations(conn, world, frame_idx, scores, gt=gt)
    touched = [s["bay_id"] for s in scores]
    deltas = db.recompute_states(conn, world, touched, frame_idx)
    conn.commit()

    if redis_client is not None and deltas:
        payload = {
            "frame_id": frame_id,
            "world": world,
            "frame_idx": frame_idx,
            "changed": [{"type": "bay_delta", **d} for d in deltas],
        }
        redis_client.publish(DELTAS_CHANNEL, json.dumps(payload))

    return {"scored": len(scores), "deltas": deltas}
