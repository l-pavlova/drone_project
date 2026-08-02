"""Environment/config for the vision worker.

Loads the nearest .env (walking up from CWD) so the server and the dev tooling
share the monorepo root .env. Existing process env always wins.
"""
import os

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

# ---- FastAPI server --------------------------------------------------------
# Kept on :4000 so the web-user vite proxy target is unchanged.
API_PORT = int(os.environ.get("API_PORT", "4000"))
# Dev-only manual occupancy toggle. OFF unless explicitly opted into: the
# /api/v1/dev/occupy|free endpoints have NO auth, so anyone who can reach the
# port could flip any bay's state. Local dev sets ENABLE_DEV_ROUTES=true in .env.
ENABLE_DEV_ROUTES = os.environ.get("ENABLE_DEV_ROUTES", "false").lower() == "true"
# Dedicated classify threads draining the in-process job queue (numpy releases
# the GIL during array ops, so these parallelise real CV work off the event loop).
CLASSIFY_THREADS = int(os.environ.get("CLASSIFY_THREADS", "4"))

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
