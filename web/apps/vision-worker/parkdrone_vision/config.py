"""Environment/config for the vision worker.

Loads the nearest .env (walking up from CWD) so the server and the dev tooling
share the monorepo root .env. Existing process env always wins.
"""
import os
import socket

HERE = os.path.dirname(os.path.abspath(__file__))
# parkdrone_vision -> vision-worker -> apps -> web -> <repo root>
REPO_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", "..", ".."))


def _load_dotenv() -> None:
    d = os.getcwd()
    while True:
        candidate = os.path.join(d, ".env")
        if os.path.isfile(candidate):
            for line in open(candidate, encoding="utf-8"):
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                key, val = key.strip(), val.strip()
                if key and key not in os.environ:
                    os.environ[key] = val
            return
        parent = os.path.dirname(d)
        if parent == d:
            return
        d = parent


_load_dotenv()

def required(name: str) -> str:
    """Read a credential-bearing setting; fail loud rather than default one.

    Credentials get no fallback value here (same rule as
    infra/docker-compose.yml's `${VAR:?}` guards): a built-in default is a
    well-known password shipped in source, and it hides a missing .env until
    something authenticates as the wrong identity. Called lazily at connect
    time, so tooling that touches neither Postgres nor S3 (e.g. the offline
    replay golden test) still imports this module fine.
    """
    val = os.environ.get(name)
    if not val:
        raise RuntimeError(
            f"{name} is not set. Copy web/.env.example to web/.env and fill it in."
        )
    return val


# Object store (frames). The server writes frame bytes here on ingest and the
# classify threads read them back; the replay driver reads local files instead.
# Endpoint/region/bucket are addresses, not secrets, so those keep defaults;
# the access/secret pair goes through required().
S3_ENDPOINT = os.environ.get("S3_ENDPOINT", "http://localhost:9000")
S3_REGION = os.environ.get("S3_REGION", "us-east-1")
S3_BUCKET = os.environ.get("S3_BUCKET", "parkdrone-frames")

# Where the sim writes output/<survey_area>/ (frames + poses.json). Used by replay.
SIM_OUTPUT_ROOT = os.environ.get(
    "SIM_OUTPUT_ROOT", os.path.join(REPO_ROOT, "sim", "output")
)

# Where generate_world.py leaves <area>.wbt and its sidecar files: the route
# (waypoint count = a mission's frames_expected, read by sim_uplink) and the
# ground-truth labels.
SIM_WORLDS_ROOT = os.environ.get(
    "SIM_WORLDS_ROOT", os.path.join(REPO_ROOT, "sim", "worlds")
)
# Eval only: when there are no labels for a survey area (a real deployment),
# observations are recorded with gt = NULL and accuracy is reported as unknown.
GROUND_TRUTH_ROOT = os.environ.get("GROUND_TRUTH_ROOT", SIM_WORLDS_ROOT)

# UAS geographical zones published by the Bulgarian CAA (ED-269 JSON). Static
# reference data read straight off disk by nofly.py — no DB table, since there
# is nothing to join it against and it changes only when the CAA republishes.
# The filename carries the edition date, so point this at the new file rather
# than overwriting the old one. Absent file = an empty zone layer, not an error.
NOFLY_FILE = os.environ.get(
    "NOFLY_FILE",
    os.path.join(REPO_ROOT, "data", "bgr_zones_30072026", "bgr_zones_30072026.json"),
)

# ---- FastAPI server --------------------------------------------------------
# Kept on :4000 so the web-user vite proxy target is unchanged.
API_PORT = int(os.environ.get("API_PORT", "4000"))
# Dev-only manual occupancy toggle. OFF unless explicitly opted into: the
# /api/v1/dev/occupy|free endpoints have NO auth, so anyone who can reach the
# port could flip any bay's state. Local dev sets ENABLE_DEV_ROUTES=true in .env.
ENABLE_DEV_ROUTES = os.environ.get("ENABLE_DEV_ROUTES", "false").lower() == "true"
# ---- admin auth (metrics) --------------------------------------------------
# `/api/v1/metrics` and `/metrics` expose fleet state, ingest rates, queue depth
# and model accuracy. They were unauthenticated like every read route, which is
# fine for a bay's occupancy and not fine for operational internals - it is a
# free map of what the system is doing and where it is stalling.
#
# Set ADMIN_API_KEY and both endpoints require `x-admin-key`. Left UNSET they
# stay open and the server says so at startup, loudly, once: this is a dev
# convenience and the warning is what stops it quietly becoming the production
# posture. It is not the finished answer either - a real admin LOGIN (sessions,
# users, audit) is still P7 work; a shared key is the smallest thing that closes
# the open door.
ADMIN_API_KEY = os.environ.get("ADMIN_API_KEY", "")

# Dedicated classify threads draining the in-process job queue (numpy releases
# the GIL during array ops, so these parallelise real CV work off the event loop).
CLASSIFY_THREADS = int(os.environ.get("CLASSIFY_THREADS", "4"))

# ---- multi-replica: job claiming (migration 0011) --------------------------
# Work is not handed to a replica, it is CLAIMED from Postgres with
# FOR UPDATE SKIP LOCKED. That is what lets N replicas share one backlog without
# classifying the same frame twice, and what lets a live replica pick up the
# work of one that died instead of waiting for it to restart.
#
# Who this replica says it is when it claims. Only ever read back by a human (or
# the ops dashboard) asking "who has this job?", so hostname:pid is enough.
REPLICA_ID = os.environ.get("REPLICA_ID") or f"{socket.gethostname()}:{os.getpid()}"
# How long the dispatcher waits for a local wake-up before polling anyway. This
# is the ONLY latency cross-replica work pays: a frame ingested by another
# replica is claimed within a tick. Against ~0.5 frames/s per drone, 1 s is
# nothing -- which is why there is no NOTIFY channel for jobs, only for deltas.
CLAIM_POLL_S = float(os.environ.get("CLAIM_POLL_S", "1.0"))
# Local prefetch, as a multiple of CLASSIFY_THREADS. Deliberately small: a
# replica that claims the whole backlog holds leases on work it will not start
# for minutes, which is today's imbalance under a new name.
CLAIM_PREFETCH = int(os.environ.get("CLAIM_PREFETCH", "2"))
# Lease length. Must exceed prefetch_depth x per-frame classify time, or a job
# still sitting in the local queue looks abandoned and a second replica takes
# it. With CLASSIFY_THREADS 4 x CLAIM_PREFETCH 2 = 8 frames queued, that is 8 x
# ~10 ms on the heuristic backend and 8 x a few seconds on the detector -- so
# 300 s is orders of magnitude of headroom either way, and the cost of being
# generous is only how long a genuinely dead replica's work waits.
LEASE_S = int(os.environ.get("LEASE_S", "300"))
# Give up after this many claims of the same job. Before 0011 a job that failed
# for any reason other than a missing image stayed queued forever and was re-run
# on every restart; this is the stop.
MAX_ATTEMPTS = int(os.environ.get("MAX_ATTEMPTS", "3"))

# ---- multi-replica: the shared delta channel -------------------------------
# Deltas cross replicas over Postgres LISTEN/NOTIFY and are ordered by
# bay_delta.id, which is the cursor clients resume from.
#
# How long the delta tail is kept. It is a replay buffer, not history --
# `observation` is the history -- so this only has to cover a browser's outage.
DELTA_RETENTION_S = int(os.environ.get("DELTA_RETENTION_S", "3600"))  # 1 h
# Cap on a single reconnect's replay. Beyond this the client reconciles from
# GET /api/v1/bays, which it already refetches on every snapshot_cursor.
DELTA_REPLAY_MAX = int(os.environ.get("DELTA_REPLAY_MAX", "500"))
# Ids are handed out by the sequence BEFORE commit, so under concurrent writers
# id 100 can become visible after id 101 and a strict `id > since` would skip
# it. A delta is an idempotent "set bay X to this state", so re-sending a few is
# free while missing one leaves a stale bay on the map. Replay a little behind
# the cursor rather than exactly at it.
DELTA_REPLAY_SLACK = int(os.environ.get("DELTA_REPLAY_SLACK", "50"))

# ---- occupancy backend -----------------------------------------------------
# Which model turns a frame into per-bay verdicts.
#
#   heuristic  vision/score_occupancy.classify() -- five colour thresholds
#              calibrated on Webots tones. The ONLY backend that reproduces the
#              simulator numbers on record, and the reason it is the default:
#              `replay fmi_block` must stay 42/42 at 100% without anyone having
#              to set a variable.
#   detector   a fine-tuned whole-frame car detector. For REAL photographs,
#              where the heuristic is meaningless -- and conversely useless on
#              rendered frames, where a COCO-scale detector finds 0 of 52 cars.
#
# The two are not interchangeable, so this is deliberately a deployment-level
# choice rather than something guessed per frame: a survey area is either
# simulated or real, and whoever starts the server knows which.
OCCUPANCY_BACKEND = os.environ.get("OCCUPANCY_BACKEND", "heuristic")
DETECTOR_WEIGHTS = os.environ.get(
    "DETECTOR_WEIGHTS",
    os.path.join(REPO_ROOT, "vision", "runs", "merged1", "weights", "best.pt"))
DETECTOR_CONF = float(os.environ.get("DETECTOR_CONF", "0.25"))
# Inference size, which MUST match what the weights were trained at (merged1:
# 1024). Ultralytics letterboxes to this, so leaving the 640 default would feed
# a 3840 px frame in at 0.17 scale -- a 385 px car arriving as 64 px, against
# the ~385 px the model learned. Silently worse, never an error.
DETECTOR_IMGSZ = int(os.environ.get("DETECTOR_IMGSZ", "1024"))
# Which camera's intrinsics to project with when the detector backend is on.
# The sim camera is 400x240 at 45 deg and the real one 3840x2160 at 73.7 deg, so
# this is not a detail -- see vision/cameras.py.
DETECTOR_CAMERA = os.environ.get("DETECTOR_CAMERA", "dji")

# ---- occupancy freshness ---------------------------------------------------
# How far back an observation counts toward a bay's occupancy vote, and how old
# a bay_state row may be before reads report it as unknown.
#
# MUST exceed the survey period. The vote is per (bay, survey_area) over this
# window, so if a patrol takes longer than the window it expires its own early
# bays before it lands — the 1 km route is 1976 waypoints, >60 min at cruise.
OCCUPANCY_WINDOW_S = int(os.environ.get("OCCUPANCY_WINDOW_S", "7200"))  # 2 h

# ---- retention (cleanup.py) ------------------------------------------------
# Frames are transient classifier input: once scored, the pose lives on in
# `observation` and the pixels are dead weight. After this age a frame's row and
# its object-store image are deleted. `observation` and `mission` are KEPT —
# they are the analytics history.
FRAME_RETENTION_S = int(os.environ.get("FRAME_RETENTION_S", "14400"))  # 4 h
# How often the background cleanup pass runs. 0 disables it (same escape hatch
# as CLASSIFY_THREADS=0), which is how the tests drive cleanup by hand.
CLEANUP_INTERVAL_S = int(os.environ.get("CLEANUP_INTERVAL_S", "900"))  # 15 min

# ---- driving directions (GET /api/v1/route -> routing.py) -------------------
# The browser never calls the router itself; we proxy, so user coordinates stay
# on our origin and the provider is swappable. Defaults to the public OSRM demo
# server (rate-limited — fine for the demo); point OSRM_BASE at a self-hosted
# OSRM for production.
OSRM_BASE = os.environ.get("OSRM_BASE", "https://router.project-osrm.org")
OSRM_TIMEOUT_S = float(os.environ.get("OSRM_TIMEOUT_S", "6"))
# The map re-routes as the driver moves, so cache on quantised coordinates
# instead of hammering the upstream router.
ROUTE_CACHE_TTL_S = float(os.environ.get("ROUTE_CACHE_TTL_S", "60"))
ROUTE_CACHE_MAX = int(os.environ.get("ROUTE_CACHE_MAX", "256"))
