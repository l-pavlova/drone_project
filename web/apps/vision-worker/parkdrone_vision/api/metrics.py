"""Operational metrics — what the admin dashboard (P6) reads to answer
"is data flowing, and is the pipeline keeping up".

Two views of one snapshot:

  * ``GET /api/v1/metrics`` → JSON, shaped for the dashboard (nested sections,
    a few list-valued extras like the active-mission table).
  * ``GET /metrics``        → Prometheus text exposition, for a scraper.

The snapshot has two halves, and the split is not arbitrary:

  * **in-process** (``processing.jobs.stats()``) — queue depth exists only in
    this process's ``queue.Queue``, and per-frame CV cost plus process-lifetime
    totals outlive the DB's retention window, so they are counted in memory.
    They reset on restart, which is correct: they describe *this* process.
    Since job claiming (0011) that distinction matters more, not less: ``queue.
    depth`` is this replica's PREFETCH, while the backlog every replica shares
    is ``jobs.queued`` below.
  * **durable** (``db.web_db``) — job outcomes, ingest times, fleet/mission
    progress, bay coverage and model accuracy where ground truth exists. Survive
    a restart; bounded by the frame retention sweep (and, for accuracy, by a
    partial index on the labelled rows), so the queries stay cheap.

No Prometheus client library: the exposition format is a few lines of text and
the dependency would buy nothing. Note the metric-type honesty — the durable
counters are windowed queries, not monotonic process counters, so they are
exported as gauges; only the in-process lifetime totals are ``counter``.

Both endpoints are unauthenticated today, like every other read route. They
expose no bay geometry and no secrets, but they do describe fleet activity, so
they belong behind the admin auth that P7 introduces.
"""
import time

from ..config import (CLASSIFY_THREADS, FRAME_RETENTION_S, OCCUPANCY_WINDOW_S,
                      REPLICA_ID)
from ..db import web_db
from ..processing import jobs

# Wall-clock process start. Module import happens during server start-up, which
# is close enough to "when did this replica come up" for an uptime reading.
_PROCESS_START = time.time()

# Default look-back for the rate/latency/active-drone windows. 5 minutes is
# short enough to show a stall quickly and long enough that a ~0.5 frames/s
# drone contributes a meaningful sample.
DEFAULT_WINDOW_S = 300


def snapshot(conn, window_s: int = DEFAULT_WINDOW_S, hub=None) -> dict:
    """Collect the full metrics payload. One connection, five small queries."""
    proc = jobs.stats()
    return {
        "generated_at": time.time(),
        "window_s": window_s,
        "process": {
            "uptime_s": round(time.time() - _PROCESS_START, 1),
            "classify_threads": CLASSIFY_THREADS,
            "workers_started": proc["workers"],
            "hub_clients": hub.client_count() if hub else 0,
            "replica_id": REPLICA_ID,
        },
        "queue": {
            "depth": proc["queue_depth"],
            "in_flight": proc["in_flight"],
            # lifetime = since this process started, not since the DB was seeded
            "processed_lifetime": proc["processed"],
            "failed_lifetime": proc["failed"],
            # Jobs taken off a replica whose lease lapsed — i.e. how much work
            # this process picked up from a dead one. Replaces the old
            # "recovered_on_start", which could only ever mean "my own".
            "reclaimed_lifetime": proc["reclaimed"],
            "deltas_pushed_lifetime": proc["deltas_pushed"],
            "avg_classify_s": proc["avg_classify_s"],
        },
        "jobs": web_db.job_counts(conn),
        "ingest": web_db.ingest_rates(conn, window_s),
        "latency": web_db.classify_latency(conn, window_s),
        "fleet": web_db.fleet_health(conn, window_s),
        "coverage": web_db.coverage_counts(conn),
        "model": web_db.model_accuracy(conn),
        "config": {
            "occupancy_window_s": OCCUPANCY_WINDOW_S,
            "frame_retention_s": FRAME_RETENTION_S,
        },
    }


# ---- Prometheus exposition -------------------------------------------------

# (metric name, type, help, path into the snapshot). Everything that is a plain
# number and useful to alert on; the mission table and timestamps are JSON-only.
_SERIES = [
    ("parkdrone_uptime_seconds", "gauge", "Seconds since this server process started", ("process", "uptime_s")),
    ("parkdrone_classify_threads", "gauge", "Configured classify threads", ("process", "classify_threads")),
    ("parkdrone_ws_clients", "gauge", "Connected occupancy WebSocket clients", ("process", "hub_clients")),
    ("parkdrone_queue_depth", "gauge", "Frames claimed by this replica and waiting on its local queue", ("queue", "depth")),
    ("parkdrone_queue_in_flight", "gauge", "Frames being classified right now", ("queue", "in_flight")),
    ("parkdrone_frames_classified_total", "counter", "Frames classified since process start", ("queue", "processed_lifetime")),
    ("parkdrone_frames_failed_total", "counter", "Classify failures since process start", ("queue", "failed_lifetime")),
    ("parkdrone_deltas_pushed_total", "counter", "Bay deltas produced by frames this process classified; delivery is via the shared channel, to whichever replica holds the client", ("queue", "deltas_pushed_lifetime")),
    ("parkdrone_classify_seconds_avg", "gauge", "Mean per-frame classify time this process", ("queue", "avg_classify_s")),
    ("parkdrone_jobs_queued", "gauge", "frame_job rows unclaimed by any replica", ("jobs", "queued")),
    ("parkdrone_jobs_running", "gauge", "frame_job rows claimed and being classified", ("jobs", "running")),
    ("parkdrone_jobs_expired_leases", "gauge", "Claimed jobs whose holder stopped renewing; a replica died mid-frame", ("jobs", "expired_leases")),
    ("parkdrone_frames_reclaimed_total", "counter", "Jobs this process took over from a lapsed lease", ("queue", "reclaimed_lifetime")),
    ("parkdrone_jobs_processed", "gauge", "frame_job rows processed (within retention)", ("jobs", "processed")),
    ("parkdrone_jobs_failed", "gauge", "frame_job rows failed (within retention)", ("jobs", "failed")),
    ("parkdrone_oldest_queued_age_seconds", "gauge", "Age of the oldest UNCLAIMED job; grows when the pipeline stalls", ("jobs", "oldest_queued_age_s")),
    ("parkdrone_frames_ingested_1m", "gauge", "Frames ingested in the last minute", ("ingest", "frames_last_1m")),
    ("parkdrone_frames_ingested_1h", "gauge", "Frames ingested in the last hour", ("ingest", "frames_last_1h")),
    ("parkdrone_frames_per_minute", "gauge", "Ingest rate over the metrics window", ("ingest", "frames_per_min_window")),
    ("parkdrone_job_latency_p50_seconds", "gauge", "Median enqueue-to-finish latency over the window", ("latency", "p50_s")),
    ("parkdrone_job_latency_p95_seconds", "gauge", "95th percentile enqueue-to-finish latency over the window", ("latency", "p95_s")),
    ("parkdrone_job_failure_rate", "gauge", "Failed share of jobs finished in the window", ("latency", "failure_rate")),
    ("parkdrone_drones_total", "gauge", "Registered drones", ("fleet", "drones_total")),
    ("parkdrone_drones_active", "gauge", "Drones seen within the metrics window", ("fleet", "drones_active")),
    ("parkdrone_missions_active", "gauge", "Missions with no ended_at", ("fleet", "missions_active")),
    ("parkdrone_bays_total", "gauge", "Bays in the database", ("coverage", "bays_total")),
    ("parkdrone_bays_occupied", "gauge", "Bays currently occupied (fresh state)", ("coverage", "bays_occupied")),
    ("parkdrone_bays_free", "gauge", "Bays currently free (fresh state)", ("coverage", "bays_free")),
    ("parkdrone_bays_unknown", "gauge", "Bays with no fresh state", ("coverage", "bays_unknown")),
    ("parkdrone_bays_closed", "gauge", "Bays under an active street closure", ("coverage", "bays_closed")),
    # Absent, not zero, where there is no ground truth to score against.
    ("parkdrone_model_state_accuracy", "gauge", "Voted bay state vs ground truth, 0-1", ("model", "state_accuracy")),
    ("parkdrone_model_view_accuracy", "gauge", "Single-view classifications vs ground truth, 0-1", ("model", "view_accuracy")),
    ("parkdrone_model_bays_scored", "gauge", "Bays carrying a ground-truth label", ("model", "bays_scored")),
    ("parkdrone_model_views_scored", "gauge", "Labelled single-view classifications", ("model", "views_scored")),
]


def prometheus(snap: dict) -> str:
    """Render a snapshot as Prometheus text exposition (v0.0.4)."""
    lines = []
    for name, kind, help_text, path in _SERIES:
        value = snap
        for key in path:
            value = value.get(key) if isinstance(value, dict) else None
        if value is None:  # "no data yet" is absence, not zero
            continue
        lines.append(f"# HELP {name} {help_text}")
        lines.append(f"# TYPE {name} {kind}")
        lines.append(f"{name} {float(value)}")
    return "\n".join(lines) + "\n"
