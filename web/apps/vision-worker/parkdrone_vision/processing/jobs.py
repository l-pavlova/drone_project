"""Frame-job pipeline: claim from Postgres, classify on local threads.

A fixed pool of dedicated OS threads does the CV. numpy releases the GIL during
the heavy array work, so they run real CV in parallel WITHOUT blocking the
FastAPI event loop. Each thread owns its own long-lived psycopg2 connection
(connections are not shareable across threads).

**Where the work comes from is what changed for multi-replica.** It used to be
pushed: the ingest handler put the job straight onto this process's
queue.Queue, and on startup `recover()` re-enqueued every row still marked
'queued' -- with no ownership filter, which is exactly why a second replica was
unsafe. Both would drain the whole backlog, classify every frame twice, and
double-count the vote.

Now work is PULLED. One dispatcher thread claims rows with
`FOR UPDATE SKIP LOCKED` (web_db.claim_frames) and feeds the same local queue:

    ingest --> frame_job row ('queued')        [committed with the frame]
                     |
    dispatcher --claim--> local queue.Queue --> classify threads
                     ^
              any replica, whoever asks first

Three consequences worth stating, because they are the design:

* **A claim is a LEASE, not an assignment.** A replica that dies mid-frame has
  its jobs reclaimed by a live one when the lease lapses -- the reaper is just
  the second arm of the claim query. `recover()` is gone: recovery stopped being
  a startup step and became something that happens continuously, to anyone.
* **The local queue is kept SHALLOW** (CLAIM_PREFETCH). Claiming the whole
  backlog would hold leases on work this replica will not start for minutes,
  which is the old imbalance under a new name.
* **Ingest no longer enqueues, it WAKES.** The frame_job row is the queue entry;
  `wake()` only spares the dispatcher its poll interval, so a frame ingested by
  another replica is picked up a tick later and nothing is lost if a wake-up is
  missed.

Deltas are not pushed from here at all any more -- process_frame publishes them
to the shared channel (processing/deltas.py) and every replica's listener thread
delivers to its own clients.
"""
import queue
import threading
import time

from botocore.exceptions import ClientError

from .. import config, s3
from ..db import vision_db, web_db
from ..vision import scoring
# `cameras` is importable because scoring.py injects <repo>/vision on sys.path
from ..vision.scoring import cameras
from . import pipeline
from .pipeline import process_frame

# S3 error codes meaning "the frame image is gone" (e.g. expired under the
# bucket's 1-day retention) — unrecoverable, so we stop retrying such frames.
_MISSING_KEY_CODES = {"NoSuchKey", "404", "NoSuchBucket"}

_q: "queue.Queue[dict]" = queue.Queue()
# Set by ingest to spare the dispatcher its poll interval; never load-bearing.
_wake = threading.Event()

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
    "reclaimed": 0,
    "classify_seconds": 0.0,
    "deltas_pushed": 0,
}


def wake() -> None:
    """Tell the dispatcher there is probably new work (ingest calls this).

    Purely a latency optimisation. The authoritative queue entry is the
    frame_job row the ingest transaction already committed, so a lost wake-up
    costs at most CLAIM_POLL_S -- which is why there is no NOTIFY channel for
    jobs and no way for this to drop work.
    """
    _wake.set()


def stats() -> dict:
    """Snapshot of the in-process pipeline counters (+ current queue depth)."""
    with _stats_lock:
        snap = dict(_stats)
    # LOCAL prefetch depth, not the backlog: work waiting for any replica
    # lives in frame_job and is reported by db/web_db.job_counts().
    snap["queue_depth"] = _q.qsize()
    done = snap["processed"]
    snap["avg_classify_s"] = round(snap["classify_seconds"] / done, 4) if done else None
    snap["classify_seconds"] = round(snap["classify_seconds"], 3)
    return snap


def _bump(**deltas) -> None:
    with _stats_lock:
        for key, val in deltas.items():
            _stats[key] += val


def start_workers(n: int, bays, index) -> None:
    for _ in range(n):
        threading.Thread(target=_worker_loop, args=(bays, index), daemon=True).start()
    _bump(workers=n)
    if n:
        threading.Thread(target=_dispatcher_loop, daemon=True).start()


def _capacity() -> int:
    """How many more jobs this replica should hold a lease on right now."""
    with _stats_lock:
        in_flight = _stats["in_flight"]
        workers = _stats["workers"]
    return max(0, workers * config.CLAIM_PREFETCH - _q.qsize() - in_flight)


def _dispatcher_loop() -> None:
    """Claim work for this replica and feed the local classify queue.

    One thread, one connection. It is deliberately the only thing that talks to
    frame_job on the way IN, so "how deep may my local queue get" lives in
    exactly one place.
    """
    conn = vision_db.connect()
    while True:
        claimed = []
        try:
            want = _capacity()
            if want:
                claimed = web_db.claim_frames(conn, config.REPLICA_ID, want,
                                              config.LEASE_S, config.MAX_ATTEMPTS)
            for job in claimed:
                _q.put(job)
            reclaimed = sum(1 for j in claimed if j.get("reclaimed"))
            if reclaimed:
                # Someone else's lease lapsed and we took it over. Worth its own
                # counter: it means a replica died mid-frame, which local queue
                # depth would never show.
                _bump(reclaimed=reclaimed)
                print(f"* reclaimed {reclaimed} abandoned frame job(s)")
        except Exception as exc:  # a claim failure must not end the dispatcher
            try:
                conn.rollback()
            except Exception:
                conn = vision_db.connect()
            print(f"! claim failed: {exc}")
        # Poll anyway when nothing woke us: that is how work ingested by ANOTHER
        # replica arrives. If we just claimed a full batch, come straight back.
        if not claimed:
            _wake.wait(config.CLAIM_POLL_S)
        _wake.clear()


def _worker_loop(bays, index) -> None:
    conn = vision_db.connect()
    # A per-THREAD detector, not a shared one: ultralytics keeps mutable
    # predictor state on the model object, so one instance across four classify
    # threads is a data race. A yolov8s is ~22 MB, which is far cheaper than the
    # lock that sharing would need -- and a lock would undo the parallelism this
    # thread pool exists for. Nothing is loaded at all on the heuristic backend.
    model = cam = None
    if pipeline.backend_name() == "detector":
        model = scoring.load_detector(config.DETECTOR_WEIGHTS)
        cam = cameras.get(config.DETECTOR_CAMERA)
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
                mission_id=job.get("mission_id"),
                model=model,
                cam=cam,
            )
            if job.get("frame_id"):
                web_db.mark_frame_processed(conn, job["frame_id"])
            # Nothing is pushed to clients from here: process_frame already
            # published these deltas to the shared channel, in the transaction
            # that produced them. This replica hears its own back through the
            # listener like everyone else, so every client sees one order.
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
                # image expired/gone: unrecoverable, so stop retrying it
                web_db.mark_frame_failed(conn, fid, exc)
                print(f"! frame {fid} image missing ({code}); marked failed")
            else:
                _give_back(conn, job, exc)
                print(f"! classify S3 error {fid}: {exc}")
        except Exception as exc:  # a bad frame must not kill the thread
            conn.rollback()
            _bump(failed=1)
            _give_back(conn, job, exc)
            print(f"! classify failed {job.get('frame_id')}: {exc}")
        finally:
            _bump(in_flight=-1)
            _q.task_done()


def _give_back(conn, job: dict, exc: Exception) -> None:
    """Release a failed job's lease, or give up on it.

    Before job claiming there was no third option: a job that failed for any
    reason other than a missing image simply stayed 'queued' and was re-run on
    every restart, forever. The claim already counted this attempt, so all that
    is decided here is whether anyone should get another one.
    """
    fid = job.get("frame_id")
    if not fid:
        return
    try:
        if job.get("attempts", 0) >= config.MAX_ATTEMPTS:
            web_db.mark_frame_failed(conn, fid, exc)
            print(f"! frame {fid} failed {config.MAX_ATTEMPTS}x; giving up")
        else:
            web_db.release_frame_job(conn, fid, exc)
    except Exception as inner:
        conn.rollback()
        print(f"! could not release job {fid}: {inner}")
