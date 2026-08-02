"""In-process frame-job pipeline.

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
import time

from botocore.exceptions import ClientError

from .. import s3
from ..db import vision_db, web_db
from .pipeline import process_frame

# S3 error codes meaning "the frame image is gone" (e.g. expired under the
# bucket's 1-day retention) — unrecoverable, so we stop retrying such frames.
_MISSING_KEY_CODES = {"NoSuchKey", "404", "NoSuchBucket"}

_q: "queue.Queue[dict]" = queue.Queue()

# Live pipeline counters for GET /api/v1/metrics. These are what Postgres cannot
# answer: queue depth is in-memory by design, and `frame_job` only keeps the last
# FRAME_RETENTION_S of history, so process-lifetime totals have to be counted
# here. Ints are updated under a lock because several classify threads share them
# (`+=` is not atomic under the GIL — it is a read-modify-write).
_stats_lock = threading.Lock()
_stats = {
    "workers": 0,
    "in_flight": 0,
    "processed": 0,
    "failed": 0,
    "recovered": 0,
    "classify_seconds": 0.0,
    "deltas_pushed": 0,
}


def enqueue(job: dict) -> None:
    _q.put(job)


def stats() -> dict:
    """Snapshot of the in-process pipeline counters (+ current queue depth)."""
    with _stats_lock:
        snap = dict(_stats)
    snap["queue_depth"] = _q.qsize()
    done = snap["processed"]
    snap["avg_classify_s"] = round(snap["classify_seconds"] / done, 4) if done else None
    snap["classify_seconds"] = round(snap["classify_seconds"], 3)
    return snap


def _bump(**deltas) -> None:
    with _stats_lock:
        for key, val in deltas.items():
            _stats[key] += val


def start_workers(n: int, bays, index, hub, loop) -> None:
    for _ in range(n):
        threading.Thread(
            target=_worker_loop, args=(bays, index, hub, loop), daemon=True
        ).start()
    _bump(workers=n)


def _worker_loop(bays, index, hub, loop) -> None:
    conn = vision_db.connect()
    while True:
        job = _q.get()
        _bump(in_flight=1)
        started = time.monotonic()
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
                index=index,
            )
            if job.get("frame_id"):
                web_db.mark_frame_processed(conn, job["frame_id"])
            if res["deltas"]:
                # bridge worker thread -> event loop; wait so errors surface and
                # deltas are flushed before the next job (broadcast is trivial).
                asyncio.run_coroutine_threadsafe(
                    hub.broadcast(res["deltas"]), loop
                ).result()
            _bump(
                processed=1,
                deltas_pushed=len(res["deltas"]),
                classify_seconds=time.monotonic() - started,
            )
        except ClientError as exc:  # image fetch failed
            conn.rollback()
            _bump(failed=1)
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
            _bump(failed=1)
            print(f"! classify failed {job.get('frame_id')}: {exc}")
        finally:
            _bump(in_flight=-1)
            _q.task_done()


def recover(conn) -> int:
    """Re-enqueue frames persisted as 'queued' but never scored (crash recovery).

    This is what makes an in-memory queue durable: it is rebuilt from Postgres
    + S3 on startup, so a restart mid-survey resumes cleanly.
    """
    jobs = web_db.unscored_frames(conn)
    for j in jobs:
        enqueue(j)
    _bump(recovered=len(jobs))
    return len(jobs)
