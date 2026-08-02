"""Expire frames whose pixels are no longer worth keeping.

    python -m parkdrone_vision.cleanup [--once]

A frame is transient classifier input. Once it has been scored, everything worth
keeping has moved on: the per-bay verdict is in `observation`, the current answer
is in `bay_state`. The row and its object-store image are then dead weight, so
after FRAME_RETENTION_S both are deleted.

What is NOT deleted:
  * `observation` — the append-only history the analytics views are built on;
  * `mission`, `bay_state`, `bay`, `drone` — small, and none of it is transient.

Two rules keep this safe:

  * **Only frames whose job reached a terminal state are collected.** A frame
    still 'queued' past the retention window is work that never ran; deleting it
    would quietly destroy data and hide a stalled pipeline. Those are counted and
    logged instead. They resolve themselves — startup recovery marks them
    'failed' once the image is gone — and the next pass then collects them.
  * **Objects go before rows.** Interrupted mid-pass, that leaves a row pointing
    at a missing image, which the classify threads already handle (see
    `_MISSING_KEY_CODES` in processing/jobs.py). The other order would leave an
    object nobody has a pointer to. The bucket's own expiry lifecycle rule is the
    backstop that eventually sweeps those.

The server runs this on a daemon thread every CLEANUP_INTERVAL_S; the CLI entry
point exists so it can also be driven by hand or from a scheduler.
"""
import sys
import threading
import time

from botocore.exceptions import ClientError

from . import s3
from .config import CLEANUP_INTERVAL_S, FRAME_RETENTION_S, S3_BUCKET
from .db import vision_db

# One batch bounds both the transaction and the S3 call: delete_objects accepts
# at most 1000 keys per request.
BATCH = 1000


def _collectable(cur, retention_s, limit):
    """Frames past retention whose classify job is done with them."""
    cur.execute(
        """SELECT f.frame_id, f.image_uri
             FROM frame f JOIN frame_job j USING (frame_id)
            WHERE f.received_at < now() - make_interval(secs => %s)
              AND j.status IN ('processed', 'failed')
            LIMIT %s""",
        (retention_s, limit),
    )
    return cur.fetchall()


def _stuck_count(cur, retention_s):
    """Frames past retention still waiting to be classified — a red flag, not garbage."""
    cur.execute(
        """SELECT count(*)
             FROM frame f JOIN frame_job j USING (frame_id)
            WHERE f.received_at < now() - make_interval(secs => %s)
              AND j.status = 'queued'""",
        (retention_s,),
    )
    return cur.fetchone()[0]


def _delete_objects(uris):
    """Remove frame images; returns how many keys the store accepted."""
    keys = []
    for uri in uris:
        if not uri or not uri.startswith("s3://"):
            continue  # replay-style local path: nothing in the bucket to remove
        _, _, rest = uri.partition("s3://")
        _bucket, _, key = rest.partition("/")
        if key:
            keys.append({"Key": key})
    if not keys:
        return 0
    try:
        s3.client().delete_objects(Bucket=S3_BUCKET, Delete={"Objects": keys})
    except ClientError as exc:
        # Losing the object is survivable — the bucket lifecycle rule sweeps it —
        # but the rows must not be deleted on the strength of a failed call.
        raise RuntimeError(f"object delete failed: {exc}") from exc
    return len(keys)


def run_once(conn, retention_s=None):
    """One full pass. Returns (frames_deleted, objects_deleted, stuck)."""
    retention_s = FRAME_RETENTION_S if retention_s is None else retention_s
    frames = objects = 0
    while True:
        with conn.cursor() as cur:
            rows = _collectable(cur, retention_s, BATCH)
            if not rows:
                break
            objects += _delete_objects([uri for _, uri in rows])
            cur.execute(
                "DELETE FROM frame WHERE frame_id = ANY(%s)",  # CASCADE takes frame_job
                ([fid for fid, _ in rows],),
            )
            frames += cur.rowcount
        conn.commit()
        if len(rows) < BATCH:
            break

    with conn.cursor() as cur:
        stuck = _stuck_count(cur, retention_s)
    conn.commit()
    return frames, objects, stuck


def _log(frames, objects, stuck):
    if frames or objects or stuck:
        msg = f"cleanup: {frames} frames, {objects} objects deleted"
        if stuck:
            msg += f" — WARNING {stuck} frame(s) past retention still queued (never classified)"
        print(msg)


def start_thread():
    """Run a pass every CLEANUP_INTERVAL_S on a daemon thread. 0 disables."""
    if CLEANUP_INTERVAL_S <= 0:
        return False

    def loop():
        conn = vision_db.connect()  # own connection, like the classify threads
        while True:
            time.sleep(CLEANUP_INTERVAL_S)
            try:
                _log(*run_once(conn))
            except Exception as exc:  # a bad pass must not kill the thread
                conn.rollback()
                print(f"! cleanup failed: {exc}")

    threading.Thread(target=loop, daemon=True).start()
    return True


def main() -> None:
    conn = vision_db.connect()
    frames, objects, stuck = run_once(conn)
    conn.close()
    print(
        f"cleanup: deleted {frames} frame rows and {objects} objects "
        f"older than {FRAME_RETENTION_S}s"
    )
    if stuck:
        print(f"WARNING: {stuck} frame(s) past retention are still queued — "
              f"never classified, so they were left in place")
        sys.exit(1)


if __name__ == "__main__":
    main()
