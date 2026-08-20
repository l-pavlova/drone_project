#!/usr/bin/env bash
#
# PARKDRONE quickstart — bring up everything needed for a full app test.
#
#   pnpm quickstart              # infra + migrate/seed + server + both UIs
#   pnpm quickstart --replay     # ...then replay a survey through the live stack
#   pnpm quickstart --no-admin   # skip the ops dashboard (--no-web skips both UIs)
#   pnpm quickstart --uplink fmi_block   # also watch a LIVE Webots flight and
#                                        # post its frames as they hit disk
#   pnpm quickstart --fly fmi_block_4st  # ...and START that flight headless too:
#                                        # one command for sim -> API -> map
#   pnpm quickstart --clear --fly fmi_block_4st   # wipe that area first, so the
#                                        # re-flight actually ingests and repaints
#   pnpm quickstart --stop       # stop the app processes AND the docker infra
#
# Re-flying an area already in the database ingests nothing (idempotent on
# frame_idx) — '--clear' wipes it first (bare '--clear' takes the area from
# --fly/--uplink). Standalone: 'pnpm clear <area>'. See scripts/clear.sh.
#
# What it starts, in dependency order:
#   1. docker compose: PostGIS (:5432) + MinIO (:9000/:9001)
#   2. schema migrations + the 1698-bay seed
#   3. the FastAPI server (:4000) — web edge + in-process vision
#   4. the driver-facing map (:5173) and the ops dashboard (:5174), both
#      proxying /api and /ws to :4000
#   5. with --fly: the sim uplink, then Webots headless on that world, so the
#      map paints itself while the drone flies
#
# It is idempotent: re-running skips what is already up (compose is declarative,
# migrations are ledgered, the seeder upserts). Ctrl-C stops the app processes
# and leaves the containers running — `--stop` takes those down too.
#
# Git Bash on Windows is the target shell (project convention), but nothing here
# is Windows-specific.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="$(cd "$ROOT/../sim" && pwd)"
RUN_DIR="$ROOT/.quickstart"          # logs + pids, gitignored
WEBOTS="${WEBOTS:-/c/Program Files/Webots/msys64/mingw64/bin/webots.exe}"
SERVER_LOG="$RUN_DIR/server.log"
WEB_LOG="$RUN_DIR/web.log"
ADMIN_LOG="$RUN_DIR/admin.log"
DRONE_ID="${DRONE_ID:-drone-1}"
REPLAY_AREA="${REPLAY_AREA:-fmi_block}"
COMPOSE=(docker compose --env-file "$ROOT/.env" -f "$ROOT/infra/docker-compose.yml")

WITH_WEB=1; WITH_ADMIN=1; WITH_SEED=1; DO_REPLAY=0; DO_STOP=0; UPLINK_AREA=""; FLY_WORLD=""
CLEAR_AREA=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-web)   WITH_WEB=0; WITH_ADMIN=0 ;;
    --no-admin) WITH_ADMIN=0 ;;
    --no-seed)  WITH_SEED=0 ;;
    --replay)   DO_REPLAY=1 ;;
    --uplink)   shift; UPLINK_AREA="${1:-}"
                [ -n "$UPLINK_AREA" ] || { echo "--uplink needs a survey area" >&2; exit 2; } ;;
    --fly)      shift; FLY_WORLD="${1:-}"
                [ -n "$FLY_WORLD" ] || { echo "--fly needs a world name" >&2; exit 2; } ;;
    # The area is optional: bare --clear means "whatever I am about to fly or
    # watch", which is the case you actually want it in.
    --clear)    if [ $# -gt 1 ] && [ "${2#-}" = "$2" ]; then shift; CLEAR_AREA="$1"
                else CLEAR_AREA="@implied"; fi ;;
    --stop)     DO_STOP=1 ;;
    -h|--help)  sed -n '2,31p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown flag: $1 (try --help)" >&2; exit 2 ;;
  esac
  shift
done
# The controller keys its output folder off the route file, so a world's survey
# area IS its name — flying implies watching that area unless told otherwise.
[ -n "$FLY_WORLD" ] && [ -z "$UPLINK_AREA" ] && UPLINK_AREA="$FLY_WORLD"
if [ "$CLEAR_AREA" = "@implied" ] && [ "$DO_STOP" = 0 ]; then
  CLEAR_AREA="${UPLINK_AREA:-}"
  [ -n "$CLEAR_AREA" ] || { echo "--clear needs an area (or use it with --fly/--uplink)" >&2; exit 2; }
fi

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

# Webots spawns its own process tree and our $! is only the pipeline subshell, so
# it is stopped by image name — the same taskkill the manual runs use.
stop_webots() {
  rm -f "$RUN_DIR/webots.pid"
  taskkill //F //IM webots-bin.exe >/dev/null 2>&1 || true
  taskkill //F //IM webotsw.exe    >/dev/null 2>&1 || true
  return 0
}

mkdir -p "$RUN_DIR"

# Validate --fly BEFORE standing anything up: a typo'd world name should cost a
# second, not a full stack start.
if [ -n "$FLY_WORLD" ] && [ "$DO_STOP" = 0 ]; then
  [ -f "$SIM_DIR/worlds/$FLY_WORLD.wbt" ] ||
    die "no such world: $SIM_DIR/worlds/$FLY_WORLD.wbt (generate it with sim/generate_world.py)"
  [ -x "$WEBOTS" ] ||
    die "Webots not found at $WEBOTS — set WEBOTS=/path/to/webots.exe"
  # A scored output folder is a verification FIXTURE: `replay.py` compares the
  # live pipeline against its occupancy_results.json, and a new flight would
  # overwrite the very frames that result was computed from. Refuse rather than
  # silently invalidate it; moving the folder aside is the explicit opt-in.
  if [ -f "$SIM_DIR/output/$FLY_WORLD/occupancy_results.json" ]; then
    die "sim/output/$FLY_WORLD/ holds a scored golden fixture (occupancy_results.json).
  Flying would overwrite the frames it was computed from. Move it aside first:
      mv sim/output/$FLY_WORLD sim/output/$FLY_WORLD.old"
  fi
  # The flight RESUMES from poses.json if the folder already has captures — that
  # is the controller's own behaviour, not something this script overrides.
  # (--clear on this same area deletes them, so it re-flies from wp0 instead.)
  if [ -f "$SIM_DIR/output/$FLY_WORLD/poses.json" ] && [ "$CLEAR_AREA" != "$FLY_WORLD" ]; then
    echo "  note: sim/output/$FLY_WORLD/ already has captures — the controller will"
    echo "        resume the patrol from there. To re-fly (and re-score) from wp0:"
    echo "            pnpm clear $FLY_WORLD"
  fi
fi

if [ "$DO_STOP" = 1 ]; then
  say "stopping app processes"
  [ -f "$RUN_DIR/webots.pid" ] && stop_webots
  stop_service server; stop_service web; stop_service admin; stop_service uplink
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

# ---- 4b. optional clear ----------------------------------------------------
# Ingest is idempotent on (drone, survey_area, frame_idx), so re-flying an area
# that is already stored posts duplicates: no classify job, no bay_state change,
# no delta, and a map that never repaints. This is the opt-in that makes a
# re-flight count. It runs AFTER the migrations (the tables must exist) and
# BEFORE the server starts, so startup recovery cannot re-enqueue jobs whose
# frames are about to be deleted.
if [ -n "$CLEAR_AREA" ]; then
  say "clearing survey area '$CLEAR_AREA'"
  (cd "$ROOT/apps/vision-worker" && "$PY" -m parkdrone_vision.clear_area "$CLEAR_AREA" --yes) ||
    die "clear failed (a scored golden fixture needs --force: 'pnpm clear $CLEAR_AREA --force')"
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

# ---- 7. optional live uplink -----------------------------------------------
# Sidecar, not part of the stack: it watches sim/output/<area>/ and posts frames
# as the flight writes them. Started here so `quickstart --uplink <area>` is all
# you need running beside Webots.
if [ -n "$UPLINK_AREA" ]; then
  stop_service uplink
  say "starting sim uplink for '$UPLINK_AREA'"
  # With --fly we know the patrol ends, so let the uplink close its mission once
  # the frames stop coming; a bare --uplink waits indefinitely instead.
  idle_args=()
  [ -n "$FLY_WORLD" ] && idle_args=(--idle-exit 120)
  ( cd "$ROOT/apps/vision-worker" &&
    API_KEY="$API_KEY" exec "$PY" -m parkdrone_vision.sim_uplink "$UPLINK_AREA" \
      "${idle_args[@]}" >"$RUN_DIR/uplink.log" 2>&1 ) &
  echo $! >"$RUN_DIR/uplink.pid"
  ok "uplink running — tail .quickstart/uplink.log (waits for the flight to start)"
fi

# ---- 8. optional Webots flight ---------------------------------------------
if [ -n "$FLY_WORLD" ]; then
  # A leftover instance takes the port and the next run hangs with zero output.
  taskkill //F //IM webots-bin.exe >/dev/null 2>&1 || true
  taskkill //F //IM webotsw.exe    >/dev/null 2>&1 || true

  say "flying '$FLY_WORLD' in Webots (headless)"
  # Webots BLOCK-buffers stdout to a file and loses it when killed, so its output
  # goes through a PIPE — the documented workaround (see CLAUDE.md). That also
  # means $! is the subshell, not webots itself, which is why stop_webots kills
  # by image name rather than by pid.
  ( cd "$SIM_DIR" &&
    "$WEBOTS" --batch --mode=fast --minimize --stdout --stderr "worlds/$FLY_WORLD.wbt" 2>&1 |
      cat >"$RUN_DIR/webots.log" ) &
  echo $! >"$RUN_DIR/webots.pid"
  ok "flight started — frames land in sim/output/$FLY_WORLD/ and stream straight to the map"
fi

# ---- 9. optional replay ----------------------------------------------------
if [ "$DO_REPLAY" = 1 ]; then
  say "replaying survey '$REPLAY_AREA' through the live stack"
  (cd "$ROOT/apps/vision-worker" && API_KEY="$API_KEY" "$PY" -m parkdrone_vision.replay_ingest "$REPLAY_AREA") || true
  echo "  (0 deltas just means the state was already correct — 'pnpm clear $REPLAY_AREA' to re-run)"
fi

cat <<EOF

  PARKDRONE is up.

$([ "$WITH_WEB" = 1 ] && echo "    driver map  http://localhost:5173")
$([ "$WITH_ADMIN" = 1 ] && echo "    ops         http://localhost:5174")
    api         http://localhost:4000/api/v1/bays
    metrics     http://localhost:4000/api/v1/metrics   (prometheus: /metrics)
    minio       http://localhost:9001
    logs        .quickstart/{server,web,admin,uplink,webots}.log

  Drive it by hand:
    curl -X POST http://localhost:4000/api/v1/dev/occupy   # needs ENABLE_DEV_ROUTES=true
    curl -X POST http://localhost:4000/api/v1/dev/free

  Re-fly a survey area (ingest is idempotent — clear it or nothing happens):
    pnpm clear <survey_area>            # or: pnpm quickstart --clear --fly <world>

  Replay a survey (ingest -> classify -> WS push):
    cd apps/vision-worker && API_KEY=\$(cat ../../.quickstart/$DRONE_ID.key) \\
      python -m parkdrone_vision.replay_ingest $REPLAY_AREA

  Ctrl-C stops the server and dashboard (containers keep running).
  'pnpm quickstart --stop' takes the containers down too.

EOF

shutdown() { echo; say "shutting down"; [ -f "$RUN_DIR/webots.pid" ] && stop_webots; stop_service server; stop_service web; stop_service admin; stop_service uplink; ok "app processes stopped"; exit 0; }
trap shutdown INT TERM

# Stay in the foreground streaming the server log — this is the thing worth
# watching during a test (ingest, classify, delta pushes all log here).
tail -f "$SERVER_LOG" &
wait $!
