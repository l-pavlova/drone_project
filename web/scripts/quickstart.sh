#!/usr/bin/env bash
#
# PARKDRONE quickstart — bring up everything needed for a full app test.
#
#   pnpm quickstart              # infra + migrate/seed + server + both UIs
#   pnpm quickstart --replay     # ...then replay a survey through the live stack
#   pnpm quickstart --no-admin   # skip the ops dashboard (--no-web skips both UIs)
#   pnpm quickstart --stop       # stop the app processes AND the docker infra
#
# What it starts, in dependency order:
#   1. docker compose: PostGIS (:5432) + MinIO (:9000/:9001)
#   2. schema migrations + the 1698-bay seed
#   3. the FastAPI server (:4000) — web edge + in-process vision
#   4. the driver-facing map (:5173) and the ops dashboard (:5174), both
#      proxying /api and /ws to :4000
#
# It is idempotent: re-running skips what is already up (compose is declarative,
# migrations are ledgered, the seeder upserts). Ctrl-C stops the app processes
# and leaves the containers running — `--stop` takes those down too.
#
# Git Bash on Windows is the target shell (project convention), but nothing here
# is Windows-specific.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="$ROOT/.quickstart"          # logs + pids, gitignored
SERVER_LOG="$RUN_DIR/server.log"
WEB_LOG="$RUN_DIR/web.log"
ADMIN_LOG="$RUN_DIR/admin.log"
DRONE_ID="${DRONE_ID:-drone-1}"
REPLAY_AREA="${REPLAY_AREA:-fmi_block}"
COMPOSE=(docker compose --env-file "$ROOT/.env" -f "$ROOT/infra/docker-compose.yml")

WITH_WEB=1; WITH_ADMIN=1; WITH_SEED=1; DO_REPLAY=0; DO_STOP=0
for arg in "$@"; do
  case "$arg" in
    --no-web)   WITH_WEB=0; WITH_ADMIN=0 ;;
    --no-admin) WITH_ADMIN=0 ;;
    --no-seed) WITH_SEED=0 ;;
    --replay)  DO_REPLAY=1 ;;
    --stop)    DO_STOP=1 ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown flag: $arg (try --help)" >&2; exit 2 ;;
  esac
done

say()  { printf '\033[36m==>\033[0m %s\n' "$*"; }
ok()   { printf '\033[32m  ok\033[0m %s\n' "$*"; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# Wait for `cmd` to succeed, up to $1 seconds. Cheaper and more honest than a
# fixed sleep: every service below has a real readiness probe.
wait_for() {
  local secs="$1" what="$2"; shift 2
  for _ in $(seq "$secs"); do
    if "$@" >/dev/null 2>&1; then ok "$what ready"; return 0; fi
    sleep 1
  done
  die "$what did not come up within ${secs}s (see $RUN_DIR/*.log)"
}

# Background a long-running service and record a pid we can actually kill.
# `exec` matters: without it, `( cd X && cmd ) &` leaves $! pointing at the
# wrapper subshell, and the real process survives every later kill.
start_service() {
  local name="$1" dir="$2" log="$3"; shift 3
  ( cd "$dir" && exec "$@" >"$log" 2>&1 ) &
  echo $! >"$RUN_DIR/$name.pid"
}

# Git Bash pids are MSYS pids; taskkill needs the Windows one (ps -W column 4).
winpid() { ps -W 2>/dev/null | awk -v p="$1" '$1 == p { print $4 }'; }

port_pids() {
  netstat -ano 2>/dev/null |
    awk -v p=":$1\$" '$1 == "TCP" && $2 ~ p && $4 == "LISTENING" { print $5 }' | sort -u
}

# Note the trailing `return 0` / `|| true` throughout: under `set -e`, a helper
# whose last command is a failed test (no such pid, nothing to kill) would abort
# the whole script — and "there was nothing to stop" is a normal outcome here.
stop_service() {
  local name="$1" f="$RUN_DIR/$1.pid" pid wp
  [ -f "$f" ] || return 0
  pid="$(cat "$f")"
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    wp="$(winpid "$pid")"
    if [ -n "$wp" ]; then taskkill //F //T //PID "$wp" >/dev/null 2>&1 || true; fi
  fi
  rm -f "$f"
  return 0
}

# The stack is only usable if it owns its ports, and a stale server from an
# earlier session (or a crashed run whose pidfile is gone) is the common case.
# Announce it rather than killing quietly.
free_port() {
  local port="$1" pid
  for pid in $(port_pids "$port"); do
    echo "  note: killing pid $pid still listening on :$port"
    taskkill //F //T //PID "$pid" >/dev/null 2>&1 || true
  done
  for _ in $(seq 10); do
    [ -z "$(port_pids "$port")" ] && return 0
    sleep 1
  done
  die "port $port is still in use"
}

mkdir -p "$RUN_DIR"

if [ "$DO_STOP" = 1 ]; then
  say "stopping app processes"
  stop_service server; stop_service web; stop_service admin
  # also catch survivors of a crashed run, whose pidfile is gone
  free_port 4000; free_port 5173; free_port 5174
  say "stopping docker infra"
  if [ -f "$ROOT/.env" ]; then "${COMPOSE[@]}" down || true; fi
  ok "stopped (docker volumes kept — 'docker compose ... down -v' wipes the data)"
  exit 0
fi

# ---- 1. .env ---------------------------------------------------------------
# The template ships placeholder passwords on purpose (nothing in the repo falls
# back to a credential). Fill them ONLY when creating .env: rewriting them later
# would desync from the password already baked into the Postgres volume.
if [ ! -f "$ROOT/.env" ]; then
  say "creating web/.env from .env.example"
  cp "$ROOT/.env.example" "$ROOT/.env"
  if command -v openssl >/dev/null 2>&1; then
    secret="$(openssl rand -hex 16)"
  else
    secret="dev$(date +%s)$RANDOM"
  fi
  sed -i "s/CHANGE_ME_local_dev/$secret/g" "$ROOT/.env"
  ok "generated local dev credentials"
else
  grep -q CHANGE_ME_local_dev "$ROOT/.env" &&
    echo "  note: web/.env still contains CHANGE_ME_local_dev placeholders" || true
fi

# ---- 2. docker infra -------------------------------------------------------
command -v docker >/dev/null 2>&1 || die "docker not found — install/start Docker Desktop"
docker info >/dev/null 2>&1 || die "Docker daemon not reachable — start Docker Desktop"

say "starting PostGIS + MinIO"
"${COMPOSE[@]}" up -d
# pg_isready inside the container: the port is open long before the DB accepts
# connections, so a TCP check would let the migration race the boot.
wait_for 90 "postgres" "${COMPOSE[@]}" exec -T postgres pg_isready -q
wait_for 60 "minio" curl -fsS http://localhost:9000/minio/health/live

# ---- 3. dependencies -------------------------------------------------------
PNPM="pnpm"
command -v pnpm >/dev/null 2>&1 || die "pnpm not found — 'npm i -g pnpm' (corepack isn't on PATH here)"
if [ ! -d "$ROOT/node_modules" ]; then
  say "installing JS dependencies"
  (cd "$ROOT" && $PNPM install)
fi

PY="${PYTHON:-python}"
command -v "$PY" >/dev/null 2>&1 || die "python not found (set PYTHON=/path/to/python)"
if ! "$PY" -c "import fastapi, uvicorn, psycopg2, boto3, numpy, PIL, multipart" >/dev/null 2>&1; then
  say "installing Python server dependencies"
  "$PY" -m pip install -q -r "$ROOT/apps/vision-worker/requirements.txt"
fi

# ---- 4. schema + seed ------------------------------------------------------
say "applying migrations"
(cd "$ROOT" && $PNPM db:migrate)
if [ "$WITH_SEED" = 1 ]; then
  say "seeding bays"
  (cd "$ROOT" && $PNPM db:seed)
fi

# ---- 5. server -------------------------------------------------------------
stop_service server
free_port 4000
say "starting server on :4000"
start_service server "$ROOT/apps/vision-worker" "$SERVER_LOG" "$PY" -m parkdrone_vision.server
wait_for 60 "server /health" curl -fsS http://localhost:4000/health

# An API key is needed for ingest (replay, or a real drone). Registering is an
# upsert, so this rotates the key on every run — fine for a dev stack, and it
# keeps the printed key true.
say "registering drone '$DRONE_ID'"
API_KEY="$( (cd "$ROOT/apps/vision-worker" && "$PY" -m parkdrone_vision.register_drone "$DRONE_ID") \
            | sed -n 's/^API key.*: //p' )"
[ -n "$API_KEY" ] || die "could not register a drone (see above)"
printf '%s' "$API_KEY" >"$RUN_DIR/$DRONE_ID.key"
ok "API key written to .quickstart/$DRONE_ID.key"

# ---- 6. dashboard ----------------------------------------------------------
if [ "$WITH_WEB" = 1 ]; then
  stop_service web
  free_port 5173
  say "starting driver map on :5173"
  start_service web "$ROOT/apps/web-user" "$WEB_LOG" "$PNPM" exec vite
  wait_for 60 "driver map" curl -fsS http://localhost:5173/
fi

if [ "$WITH_ADMIN" = 1 ]; then
  stop_service admin
  free_port 5174
  say "starting ops dashboard on :5174"
  start_service admin "$ROOT/apps/web-admin" "$ADMIN_LOG" "$PNPM" exec vite
  wait_for 60 "ops dashboard" curl -fsS http://localhost:5174/
fi

# ---- 7. optional replay ----------------------------------------------------
if [ "$DO_REPLAY" = 1 ]; then
  say "replaying survey '$REPLAY_AREA' through the live stack"
  (cd "$ROOT/apps/vision-worker" && API_KEY="$API_KEY" "$PY" -m parkdrone_vision.replay_ingest "$REPLAY_AREA") || true
  echo "  (0 deltas just means the state was already correct — clear frame/observation/bay_state to re-run)"
fi

cat <<EOF

  PARKDRONE is up.

$([ "$WITH_WEB" = 1 ] && echo "    driver map  http://localhost:5173")
$([ "$WITH_ADMIN" = 1 ] && echo "    ops         http://localhost:5174")
    api         http://localhost:4000/api/v1/bays
    metrics     http://localhost:4000/api/v1/metrics   (prometheus: /metrics)
    minio       http://localhost:9001
    logs        .quickstart/{server,web,admin}.log

  Drive it by hand:
    curl -X POST http://localhost:4000/api/v1/dev/occupy   # needs ENABLE_DEV_ROUTES=true
    curl -X POST http://localhost:4000/api/v1/dev/free

  Replay a survey (ingest -> classify -> WS push):
    cd apps/vision-worker && API_KEY=\$(cat ../../.quickstart/$DRONE_ID.key) \\
      python -m parkdrone_vision.replay_ingest $REPLAY_AREA

  Ctrl-C stops the server and dashboard (containers keep running).
  'pnpm quickstart --stop' takes the containers down too.

EOF

shutdown() { echo; say "shutting down"; stop_service server; stop_service web; stop_service admin; ok "app processes stopped"; exit 0; }
trap shutdown INT TERM

# Stay in the foreground streaming the server log — this is the thing worth
# watching during a test (ingest, classify, delta pushes all log here).
tail -f "$SERVER_LOG" &
wait $!
