"""Environment/config for the vision worker.

Loads the nearest .env (walking up from CWD) so the worker shares the monorepo
root .env with the Node services. Existing process env always wins.
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

DATABASE_URL = os.environ.get(
    "DATABASE_URL", "postgres://parkdrone:parkdrone@localhost:5432/parkdrone"
)

# Object store (frames). The server writes frame bytes here on ingest and the
# classify threads read them back; the replay driver reads local files instead.
S3_ENDPOINT = os.environ.get("S3_ENDPOINT", "http://localhost:9000")
S3_REGION = os.environ.get("S3_REGION", "us-east-1")
S3_BUCKET = os.environ.get("S3_BUCKET", "parkdrone-frames")
S3_ACCESS_KEY = os.environ.get("S3_ACCESS_KEY", "minioadmin")
S3_SECRET_KEY = os.environ.get("S3_SECRET_KEY", "minioadmin")

# Where the sim writes output/<survey_area>/ (frames + poses.json). Used by replay.
SIM_OUTPUT_ROOT = os.environ.get(
    "SIM_OUTPUT_ROOT", os.path.join(REPO_ROOT, "sim", "output")
)

# ---- FastAPI server (the monolith replacing the Node API + Redis) ----------
# Kept on :4000 so the web-user vite proxy target is unchanged.
API_PORT = int(os.environ.get("API_PORT", "4000"))
# Dev-only manual occupancy toggle; mount unless explicitly disabled.
ENABLE_DEV_ROUTES = os.environ.get("ENABLE_DEV_ROUTES", "true").lower() != "false"
# Dedicated classify threads draining the in-process job queue (numpy releases
# the GIL during array ops, so these parallelise real CV work off the event loop).
CLASSIFY_THREADS = int(os.environ.get("CLASSIFY_THREADS", "4"))

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
