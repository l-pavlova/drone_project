# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

PARKDRONE: an autonomous drone that surveys **parking-space occupancy** (occupied vs. free) over a Sofia city block. `docs/PARKDRONE_FULL_MASTER_PLAN*.md` describes the full hardware vision (Pixhawk + Raspberry Pi 4 + Coral USB + IMX219 camera, GPS nav, obstacle avoidance, AI detection), but the **active work is in simulation**: fly a lawnmower patrol in Webots over a real-data parking block, capture downward camera frames, and (next) detect cars and score occupancy against ground truth.

There is no build system and no test suite — the code is standalone Python scripts plus one Webots robot controller. "Running tests" means running the data scripts or the sim headless and inspecting their output.

## Pipeline & commands

Three stages. Stages 1–2 are built and working; stage 3 (`vision/`) is not started.

### 1. GIS data prep (run from `tools/`, writes to `data/`)
```bash
python build_demo_area.py ["Лозенец"]     # clip spaces_25.geojson to an OSM district boundary
python cut_block.py "<address or lat,lon>" 200   # cut a square block (200 = half-size m) -> block_spaces.geojson
python make_bays.py                        # space POINTS -> oriented bay RECTANGLES -> block_bays.geojson
python get_roads.py                        # OSM street centerlines for the block (Overpass) -> block_roads.geojson
```
`spaces_25.geojson` (31,705 parking-space points) and `zones_34.geojson` are the source datasets from the Sofiaplan API; see `data/README.md` for schema and field meanings (Bulgarian property values).

### 2. Webots simulation (run from `sim/`)
```bash
python generate_world.py [half_m] [occ_frac]   # default 75 0.5 -> worlds/fmi_block.wbt + worlds/ground_truth.json
```
This reads `../data/block_bays.geojson` (plus `block_roads.geojson` if present), projects to local metres, and emits the world (ground, OSM streets as Webots `Road` protos, painted bays, real car models — 7 vehicle Simple protos — on a known-occupancy subset, follow-drone viewpoint) plus `ground_truth.json` (bay_id -> occupied). Car/model randoms come from a separate `random.Random(7)` stream so `ground_truth.json` stays stable. `DirectionalLight` has `castShadows FALSE` — shadow mapping paints streak artifacts on the road/ground in the nadir frames. Then run the world (see "Running Webots" below). The route (`worlds/route.json`) is an open-path rural-postman walk of the OSM street centerlines of every street that has bays: disconnected coverage components are joined by shortest road transits (MST), odd-degree nodes are evened out with a minimum-weight matching (exact blossom if `networkx` is installed, stdlib fallback otherwise; two virtual endpoints make it an open path whose start is the endpoint nearest the origin), then a Hierholzer Euler walk flies every coverage edge once with deadheads only along the matched repeats — waypoints every 10 m because the camera footprint at 30 m is only ~25×15 m. The controller `controllers/parkdrone/parkdrone.py` takes off to 30 m, flies that route (square-lawnmower fallback if route.json is missing), and writes `output/<survey_area>/frame_###.png` + `output/<survey_area>/poses.json` at each waypoint (plus timed diagnostic `snap_###.png`). Worlds are per-scale file sets: `generate_world.py [half_m] [occ_frac] [name]` writes `<name>.wbt` + `<name>.route.json` + `<name>.ground_truth.json` (default name `fmi_block` keeps legacy `route.json`/`ground_truth.json`); the `.wbt` passes its route file to the controller via `controllerArgs`, which also keys the output subfolder. E.g. the 1 km world: `python generate_world.py 500 0.5 fmi_block_1km`. The local-metre frame is pinned by `ORIGIN` in `generate_world.py` — do NOT let it drift when re-cutting data at other sizes.

### 3. Flight-log analysis (real-flight debugging, separate from sim)
```bash
pip install pymavlink
python tools/analyze_log.py logs/your_flight.bin   # ArduPilot .bin: modes, GPS/EKF, commanded vs actual attitude
```

## Running Webots

Webots is installed at `C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`. To run a world headless and capture controller output:
```bash
"/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" --batch --mode=fast --minimize --stdout --stderr worlds/fmi_block.wbt 2>&1 | grep -aE "reached|complete|t=" | head -1500
```
Critical run-time gotchas (each cost real debugging time):
- **Capture stdout by PIPING, not file redirect.** Webots block-buffers stdout to a file and loses it when the process is killed (timeout). Piping to `grep`/`head` flushes line-by-line; `head -N` also stops the otherwise-infinite controller loop.
- **Kill stray Webots first.** A leftover instance causes a port conflict and the next run hangs with zero output: `taskkill //F //IM webots-bin.exe; taskkill //F //IM webotsw.exe; taskkill //F //IM python.exe`.
- The controller writes `frame_###.png` / `poses.json` to `sim/output/<survey_area>/` (survey area name derived from the route file passed in `controllerArgs`), so disk output is the reliable source of truth even if console is lost. Pre-2026-07-04 runs live loose in `sim/output/`.
- `EXTERNPROTO` for `Mavic2Pro.proto` is pinned to **R2023b** via `WEBOTS_VER` in `generate_world.py`; change it if your Webots release differs, then regenerate the world. Controller device names (`camera`, `inertial unit`, `gps`, `gyro`, `camera roll`, `camera pitch`, `front/rear left/right propeller`) assume that proto.

## Architecture (the big picture)

**Coordinate flow / georeferencing is the spine of the project.** Everything is tied together by a single local projection: lon/lat (EPSG:4326) → local ENU metres about the block centroid, using `mlat = 111320`, `mlon = 111320*cos(lat0)`. Both `generate_world.py` and the controller use this same convention. `poses.json` records the drone's `x, y, alt, yaw` in those metres at each captured frame — so a detected car's image position can be projected to ground metres and matched to the nearest bay in `block_bays.geojson`, then scored against `ground_truth.json`. When touching projection math, keep `generate_world.py`, the controller, and (future) the detector consistent.

**`parkdrone.py` is one control loop.** Each step: read IMU/GPS/gyro → point gimbal to nadir → optionally capture a frame → compute roll/pitch/yaw/vertical disturbances → mix into four propeller velocities (mixing & base gains adapted from Webots' official Mavic2Pro sample). On top of the stock stabilizer sits a **lawnmower waypoint navigator** that steers car-style: yaw to point the nose at the next waypoint, then throttle forward.

Hard-won controller invariants — **do not regress these** (they are why the sim works now; details in the project memory):
- **Camera nadir uses POSITIVE pitch.** The Mavic2Pro `camera pitch` range is ~`[-0.5, +1.7]` rad where **down is positive**; set `+pi/2` for true nadir. `getMinPosition()` (-0.5) points the camera *up* and yields sky-only frames.
- **Altitude needs vertical-velocity damping** (`K_VD`), or it overshoots ~30→50 m and crashes.
- **Yaw needs rate damping** (`K_YAWD`, from the gyro's yaw rate), or it is pure-proportional and the drone spins in circles, never facing a waypoint.
- **Steer with yaw + forward only; never roll-strafe toward the target** — a lateral *position* command saturates while off-heading and tumbles the drone. Roll is used only to damp sideways drift.
- **Waypoint arrival: keep `WP_REACH` at 6 m and capture at closest approach.** At cruise speed the turn radius is ~4 m, so the drone can settle into a stable ORBIT inside a tighter basin (constant distance — the patrol hangs forever, circling). Arrival fires on `WP_CAPTURE` (2.5 m), receding >1 m past the closest pass, or a `WP_TIMEOUT` (8 s) orbit bail-out.
- **`TILT_MAX` > ~1.0 dips lift and crashes.** Forward speed is a velocity-target controller (`v_des = clamp(K_POS*fwd_err, 0, V_MAX)`) that ramps down on approach so row-end U-turns stay tight; once the patrol finishes the controller station-keeps (brakes drift) instead of sailing off.
- Working gains live at the top of the loop: `K_YAW=1.0 K_YAWD=0.8 K_POS=0.6 K_VEL=0.4 TILT_MAX=1.0 V_MAX=2.5` (plus the Webots-sample stabilizer gains `K_VT/K_VP/K_ROLL/K_PITCH`).

## Windows / Git Bash conventions

- The shell is Git Bash. **Windows backslash paths break** in commands — use forward slashes (`/c/Users/...`) or quote carefully.
- The data scripts deliberately use Python `urllib` + UTF-8 (`sys.stdout.reconfigure(encoding="utf-8")`) instead of shelling out, because **Git Bash mangles Cyrillic** (street names, zone types are in Bulgarian). For manual downloads use `curl --ssl-no-revoke` (Windows cert revocation is flaky); the scripts already disable cert verification for these public read-only GETs.

## Web infrastructure (`web/`) — live occupancy product

A separate stage 4 turns the on-disk occupancy report into a live product: an ingest API, a
real-time occupancy push, and an end-user parking map. It lives in **`web/`** (a pnpm monorepo),
independent of the Python sim/vision code. Full design in `docs/web_infra_plan.md`.
Diagrams: `web/docs/architecture.drawio` (system level), `docs/server_modules.md` +
`docs/server_modules.drawio` (inside the server), `docs/db_schema_er.md` (schema).

**Status: Phases 1–5 built & verified end-to-end; Phase 6 (admin dashboard) and Phase 7 (prod
hardening) remain.** See project memory `project-web-infra.md` for the running log.

### Layout
- `packages/contracts` — shared TS types + zod schemas + the ENU projection (mirrors
  `generate_world.py`/`score_occupancy.py`; **must** stay in lockstep — same ORIGIN/MLAT/MLON).
- `packages/db` — Postgres+PostGIS migrations and the geojson→`bay` seeder (dev tooling, run via
  `pnpm db:migrate`/`db:seed`; not on the runtime path).
- `apps/vision-worker` (Python) — **the whole server** (FastAPI monolith), organized by concern
  into subpackages: `api/` (routes in `app.py`, drone auth, the WebSocket `hub.py`), `processing/`
  (`jobs.py`'s in-process queue + classify threads, `pipeline.py`'s per-frame `process_frame`),
  `db/` (`pool.py`'s connection pool, `vision_db.py` for bay geometry/observations/bay_state,
  `web_db.py` for reads/ingest/mission/auth SQL), and `vision/scoring.py` (the classifier bridge).
  `config.py`, `s3.py`, and the CLI entry points (`server.py`, `replay.py`, `replay_ingest.py`,
  `register_drone.py`) stay at the package root. Reuses `vision/score_occupancy.py`'s
  `project`/`bay_features`/`classify` **verbatim** (via `vision/scoring.py`/`processing/pipeline.py`),
  and adds the web edge: ingest (`POST /api/v1/ingest/frame`, per-drone API key), read
  (`/api/v1/bays` GeoJSON, `/summary`, `/bays/:id`), `WS /ws/occupancy` push, and the dev toggle.
  Ingest → in-process `queue.Queue` → classify threads → `bay_state` + direct WebSocket push.
  Entry point `parkdrone_vision.server` (uvicorn on :4000, serving `parkdrone_vision.api.app:app`).
- `apps/web-user` (React + react-leaflet) — the parking map (drone "survey-readout" UI identity).
  Styling convention: **CSS Modules, one `Component.module.css` per component** (no shared
  per-component classes in `styles/global.css` — that file is trimmed to CSS variables/reset/base
  sizing only). Use `:global(...)` only for classes owned by a third party we don't render
  ourselves (e.g. Leaflet's injected `.leaflet-popup-content`).
- `infra/docker-compose.yml` — postgis + minio.

### Architecture invariants (do not regress)
- **Postgres + PostGIS from the start** (no SQLite). Bay geometry is WGS84; bbox/nearest queries
  push down into PostGIS. The ENU projection is only for pose math.
- **One Python process owns everything** (web edge *and* CV) because the classifier is Python and
  reused verbatim. No cross-language boundary, so **no broker**: the job queue is an in-process
  `queue.Queue` drained by dedicated classify threads (numpy releases the GIL, so real parallelism
  off the event loop), and deltas are pushed **straight** to WebSocket clients the same process
  holds. Read/ingest handlers are sync `def` (Starlette threadpool) so blocking psycopg2/boto3 never
  touch the loop; only the WS endpoint is async.
- **Durability without a broker:** an in-memory queue loses in-flight jobs on restart, so on startup
  the server re-enqueues jobs with `status='queued'` (rebuilt by joining `frame_job` to `frame`,
  plus S3). A frame whose image has expired from the store is marked `failed` so recovery won't
  loop on it.
- **`frame` and `frame_job` are one thing each.** `frame` is the append-only ingest ledger (pose,
  payload pointer, provenance — never UPDATEd); `frame_job` is the 1:1 classify work state
  (`status`, `enqueued_at`, `finished_at`). Both are written in one transaction before the job is
  enqueued; a duplicate ingest creates no job row. `ON DELETE CASCADE` means clearing a survey
  area's `frame` rows still clears its jobs.
- **Bay ids: int in `block_bays.geojson`, string everywhere in the web tier**.
- Frame ingest is idempotent on `UNIQUE(drone_id, survey_area, frame_idx)` — a re-send does NOT
  re-enqueue; to reprocess a survey area *within the retention window*, clear the `frame` table
  first. Past `FRAME_RETENTION_S` the rows are gone anyway, so a re-send is ingested as new.
- **Occupancy is a vote over a freshness window, not over all history** (`OCCUPANCY_WINDOW_S`,
  default 2 h). Only observations inside the window count, and a `bay_state` row older than it is
  reported as `occupied: null` (unknown) by every read path — derived at read time, not swept. The
  window **must exceed the survey period**, or a long patrol expires its own early bays before it
  lands (the 1 km route is >60 min).
- **Frames are transient.** `cleanup.py` deletes `frame` rows and their stored images past
  `FRAME_RETENTION_S` (4 h); `observation` and `mission` are kept as the analytics history. It
  refuses to collect a frame whose job is still `queued` — that is unclassified work, and dropping
  it silently would hide a stalled pipeline.
- **Single replica is a correctness requirement, not a preference.** `jobs.recover()` re-enqueues
  every `status='queued'` row with no ownership filter, so two replicas would both classify the
  same backlog and double-count the vote; the WebSocket hub and its replay cursor are also
  per-process. Scaling out needs job claiming, a shared delta channel and a global cursor first —
  see `docs/web_infra_plan.md`. Throughput is not the reason to: ~103 frames/s per classify thread
  against ~0.5 frames/s per drone.
- The server classifies **all** visible bays (production has no ground truth); `gt` is eval-only.
- `score_frame` takes an optional `BayIndex` and rejects bays outside the camera footprint before
  projecting them (footprint half-width is `alt*tan(FOV/2)`). Results are identical — the test is
  conservative — but per-frame cost stops scaling with the size of the bay dataset.

### Run it (dev)
One command brings the whole stack up for a full-app test — infra, migrations, seed, server,
dashboard, plus a registered dev drone whose API key lands in `web/.quickstart/drone-1.key`:
```bash
cd web && npm i -g pnpm    # corepack isn't on PATH here
pnpm quickstart            # add --replay to also drive a survey through the live stack,
                           # --no-web to skip the dashboard, --stop to tear everything down
```
It is idempotent and self-healing: it creates `.env` with generated credentials on first run,
waits on real readiness probes (`pg_isready`, MinIO health, `/health`), and kills whatever stale
process is still holding :4000 / :5173 (announcing it). Ctrl-C stops the server and dashboard and
leaves the containers up; logs are in `web/.quickstart/`.

The manual equivalent, step by step:
```bash
cd web && cp -n .env.example .env   # REQUIRED: compose has no baked-in credentials,
                                    # it interpolates POSTGRES_*/S3_* from .env and
                                    # fails loud if they're unset
pnpm install
pnpm infra:up           # postgis + minio (needs Docker Desktop running)
pnpm db:migrate && pnpm db:seed         # loads all 1698 bays
# Server (:4000) — the whole web edge + in-process vision, one uvicorn process.
# Needs fastapi/uvicorn/websockets/python-multipart + numpy/Pillow/psycopg2/boto3
# (pip install -r apps/vision-worker/requirements.txt):
(cd apps/vision-worker && python -m parkdrone_vision.server &)
# Dashboard (:5173, proxies /api + /ws to :4000):
(cd apps/web-user && pnpm exec vite &)
```
Verification harnesses (all Python, run from `apps/vision-worker`):
- Vision golden test: `python -m parkdrone_vision.replay fmi_block`
  (expect 43/43 match vs `occupancy_results.json`, 100% vs GT). Imports the classifier only — no
  server needed.
- Full stack E2E: register a drone `python -m parkdrone_vision.register_drone drone-1`, then
  `API_KEY=<key> python -m parkdrone_vision.replay_ingest fmi_block` (expect 52 WS deltas + final
  `/bays` matching the offline result, 43/43). To re-run, clear the survey area's `frame`,
  `observation` **and `bay_state`** rows first: `frame` because idempotency skips duplicates, and
  the other two because deltas only fire on a *change* — replay straight after the golden test
  leaves the state already correct and reports a green "0 deltas".
- Frame retention: `python -m parkdrone_vision.cleanup` runs one sweep by hand (the server also
  runs it every `CLEANUP_INTERVAL_S`; set that to 0 to disable). Exits 1 if it found frames past
  retention still queued, so a scheduler surfaces a stalled pipeline.

### Operational metrics (`api/metrics.py`, the P6 admin data source)
`GET /api/v1/metrics?window_s=300` (JSON) and `GET /metrics` (Prometheus text, no client library)
render one snapshot with two halves: **in-process** counters from `processing.jobs.stats()` — queue
depth and in-flight, which exist only in this process's `queue.Queue`, plus lifetime
classified/failed/recovered/deltas and mean classify time (they reset per process, by design) — and
**durable** queries in `db/web_db.py`: `frame_job` status counts + `oldest_queued_age_s` (the stall
signal), ingest rates, enqueue→finish latency avg/p50/p95 + failure rate, fleet/active-mission
progress, and bay coverage. Both endpoints are unauthenticated like every other read route; they go
behind admin auth in P7. To see a stall by hand: run the server with `CLASSIFY_THREADS=0`, ingest,
and watch `jobs.queued` / `oldest_queued_age_s` climb; restarting normally then shows
`recovered_on_start` and drains it.

### Dev/test occupancy toggle (drive the dashboard by hand)
Manual override endpoints (mounted only when `ENABLE_DEV_ROUTES=true` — they have no auth, so the
default is off; `.env.example` opts local dev in) upsert `bay_state` and push a
delta straight to the WebSocket hub, so the map updates live — no drone/vision needed:
```bash
curl -X POST http://localhost:4000/api/v1/dev/occupy   # occupy the bay nearest FMI (default 17596)
curl -X POST http://localhost:4000/api/v1/dev/free     # free it again
curl -X POST "http://localhost:4000/api/v1/dev/occupy?bay_id=17571"   # target a specific bay
```
Both return `{bay_id, occupied, updated_at}`; watch the bay flip red/green on :5173.

Gotchas: native Windows Python needs `D:/...` paths, not Git Bash `/d/...`. The server binds :4000;
free it by PID (`netstat -ano | grep :4000` → `taskkill //F //PID <pid>`) rather than blanket-killing
`python.exe` (also kills sim Python). `CLASSIFY_THREADS=0` starts the server without draining the
queue (used to stage frames for the restart-recovery test).
