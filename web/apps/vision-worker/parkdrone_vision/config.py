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
REDIS_URL = os.environ.get("REDIS_URL", "redis://localhost:6379")

# Redis keys shared with the Node API (see apps/api). A plain list is the job
# queue (language-agnostic, unlike BullMQ's internal format); a pub/sub channel
# carries deltas back for WebSocket fan-out.
JOBS_QUEUE = os.environ.get("PARKDRONE_JOBS_QUEUE", "parkdrone:jobs")
DELTAS_CHANNEL = os.environ.get("PARKDRONE_DELTAS_CHANNEL", "parkdrone:deltas")

# Object store (frames). Only used by the live worker when a job carries an
# s3:// image_uri; the replay driver reads local files instead.
S3_ENDPOINT = os.environ.get("S3_ENDPOINT", "http://localhost:9000")
S3_REGION = os.environ.get("S3_REGION", "us-east-1")
S3_BUCKET = os.environ.get("S3_BUCKET", "parkdrone-frames")
S3_ACCESS_KEY = os.environ.get("S3_ACCESS_KEY", "minioadmin")
S3_SECRET_KEY = os.environ.get("S3_SECRET_KEY", "minioadmin")

# Where the sim writes output/<world>/ (frames + poses.json). Used by replay.
SIM_OUTPUT_ROOT = os.environ.get(
    "SIM_OUTPUT_ROOT", os.path.join(REPO_ROOT, "sim", "output")
)
