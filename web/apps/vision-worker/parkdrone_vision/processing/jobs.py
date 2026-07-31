"""In-process frame-job pipeline (replaces the Redis list queue + worker pod).

A thread-safe queue.Queue holds classify jobs; a fixed pool of dedicated OS
threads drains it. numpy releases the GIL during the heavy array work, so these
threads do real CV in parallel WITHOUT blocking the FastAPI event loop (which
only ever touches the queue via a non-blocking put). Each thread owns its own
long-lived psycopg2 connection (connections are not shareable across threads).

Deltas produced by process_frame are pushed to WebSocket clients by scheduling
hub.broadcast on the server loop from the worker thread.
"""
import asyncio
import queue
import threading

from botocore.exceptions import ClientError

from .. import s3
from ..db import vision_db, web_db
from .pipeline import process_frame

# S3 error codes meaning "the frame image is gone" (e.g. expired under the
# bucket's 1-day retention) — unrecoverable, so we stop retrying such frames.
_MISSING_KEY_CODES = {"NoSuchKey", "404", "NoSuchBucket"}

_q: "queue.Queue[dict]" = queue.Queue()


def enqueue(job: dict) -> None:
    _q.put(job)


def start_workers(n: int, bays, hub, loop) -> None:
    for _ in range(n):
        threading.Thread(
            target=_worker_loop, args=(bays, hub, loop), daemon=True
        ).start()


def _worker_loop(bays, hub, loop) -> None:
    conn = vision_db.connect()
    while True:
        job = _q.get()
        try:
            img = s3.get_frame_array(job.get("image_path") or job["image_uri"])
            res = process_frame(
                conn,
                job["survey_area"],
                job["frame_idx"],
                img,
                job["pose"],
                bays,
                frame_id=job.get("frame_id"),
            )
            if job.get("frame_id"):
                web_db.mark_frame_processed(conn, job["frame_id"])
            if res["deltas"]:
                # bridge worker thread -> event loop; wait so errors surface and
                # deltas are flushed before the next job (broadcast is trivial).
                asyncio.run_coroutine_threadsafe(
                    hub.broadcast(res["deltas"]), loop
                ).result()
        except ClientError as exc:  # image fetch failed
            conn.rollback()
            fid = job.get("frame_id")
            code = exc.response.get("Error", {}).get("Code")
            if fid and code in _MISSING_KEY_CODES:
                # image expired/gone: mark failed so recovery won't loop on it
                web_db.mark_frame_failed(conn, fid)
                print(f"! frame {fid} image missing ({code}); marked failed")
            else:
                print(f"! classify S3 error {fid}: {exc}")
        except Exception as exc:  # a bad frame must not kill the thread
            conn.rollback()
            print(f"! classify failed {job.get('frame_id')}: {exc}")
        finally:
            _q.task_done()


def recover(conn) -> int:
    """Re-enqueue frames persisted as 'queued' but never scored (crash recovery).

    Replaces Redis's at-least-once durability: the in-memory queue is rebuilt
    from Postgres + S3 on startup, so a restart mid-survey resumes cleanly.
    """
    jobs = web_db.unscored_frames(conn)
    for j in jobs:
        enqueue(j)
    return len(jobs)
