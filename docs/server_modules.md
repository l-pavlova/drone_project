# PARKDRONE server — module map

The server is **one Python process** (`web/apps/vision-worker/parkdrone_vision`) that owns
both the web edge and the vision CV: FastAPI on `:4000` plus a pool of classify threads.
There is no broker and no second language — that is *why* the module layout looks the way it
does (see "Why it is split this way" below).

This file zooms **inside** the "PARKDRONE Server" component of
`web/docs/architecture.drawio`. An editable draw.io version of the same diagram lives next to
this file: **`docs/server_modules.drawio`**.

## Module diagram

```mermaid
flowchart TB
    drone["🛩 Drone / sim<br/>frames + poses"]
    browser["🖥 web-user<br/>React + Leaflet"]

    subgraph proc["parkdrone_vision — single process, :4000"]
        direction TB

        subgraph api["api/ — HTTP + WebSocket edge"]
            app["<b>app.py</b><br/>FastAPI app + lifespan<br/>ingest · reads · /route · metrics · dev toggle · WS<br/><i>sync handlers → threadpool</i>"]
            auth["<b>auth.py</b><br/>require_drone dependency<br/>x-api-key → SHA-256 → drone_id"]
            hub["<b>hub.py</b><br/>WS fan-out + replay buffer<br/>?since=&lt;cursor&gt; catch-up"]
            met["<b>metrics.py</b><br/>snapshot(): in-process counters + DB health<br/>JSON + Prometheus rendering"]
        end

        subgraph processing["processing/ — the classify pipeline"]
            jobs["<b>jobs.py</b><br/>queue.Queue + N classify threads<br/>recover() re-enqueues on boot"]
            pipeline["<b>pipeline.py</b><br/>process_frame(): score → observe<br/>→ recompute state → deltas"]
        end

        subgraph vision["vision/ — classifier bridge"]
            scoring["<b>scoring.py</b><br/>score_frame() per-bay over one frame<br/>re-exports project / bay_features / classify"]
            gt["<b>ground_truth.py</b> — eval only<br/>labels_for(area) → sim/worlds/*.ground_truth.json<br/>no file → gt NULL → accuracy unknown"]
        end

        subgraph dbp["db/ — all SQL"]
            pool["<b>pool.py</b><br/>ThreadedConnectionPool<br/>borrow(commit=…)"]
            webdb["<b>web_db.py</b><br/>reads (GeoJSON/summary/detail)<br/>ingest · mission · auth SQL"]
            visdb["<b>vision_db.py</b><br/>bays→ENU · observations<br/>recompute_states → deltas"]
        end

        config["<b>config.py</b><br/>.env walk-up<br/>DB / S3 / port / threads / OSRM"]
        s3m["<b>s3.py</b><br/>put_frame / get_frame_array"]
        routing["<b>routing.py</b><br/>OSRM proxy + TTL cache"]
    end

    pg[("Postgres + PostGIS<br/>system of record")]
    minio[("MinIO / S3<br/>frame pixels, 1-day TTL")]
    osrm["OSRM<br/>driving directions"]
    so["../vision/score_occupancy.py<br/><i>imported verbatim</i>"]

    drone -->|"POST /ingest/frame"| app
    browser -->|"GET /bays /summary /route"| app
    browser <-->|"WS /ws/occupancy"| hub

    app --> auth
    app -->|"put_frame"| s3m
    app -->|"insert_frame + frame_job<br/>(one tx)"| webdb
    app -->|"enqueue(job)"| jobs
    app --> routing
    app -.->|"borrow()"| pool
    auth -.->|"borrow()"| pool

    jobs -->|"get_frame_array"| s3m
    jobs --> pipeline
    jobs -->|"run_coroutine_threadsafe<br/>(broadcast)"| hub
    jobs -->|"mark processed/failed<br/>unscored_frames()"| webdb
    pipeline --> scoring
    pipeline -->|"labels_for(area)"| gt
    pipeline --> visdb
    scoring --> so

    webdb --> pg
    visdb --> pg
    pool --> pg
    s3m --> minio
    routing --> osrm

    config -.-> pool
    config -.-> s3m
    config -.-> routing

    app --> met
    met -.->|"stats(): queue depth,<br/>lifetime totals"| jobs
    met -->|"job counts · ingest rates<br/>latency · fleet · coverage"| webdb
```

Entry points (package root, not on the request path):

| Command | Module | What it does |
| --- | --- | --- |
| `python -m parkdrone_vision.server` | `server.py` | boots uvicorn on `API_PORT` with `api.app:app` |
| `python -m parkdrone_vision.register_drone <id>` | `register_drone.py` | mints an API key, stores **only** its SHA-256 |
| `python -m parkdrone_vision.replay <area>` | `replay.py` | golden test: frames → `pipeline.process_frame` → compare to `occupancy_results.json` (no server) |
| `API_KEY=… python -m parkdrone_vision.replay_ingest <area>` | `replay_ingest.py` | full-stack E2E over the real HTTP/WS contract |
| `API_KEY=… python -m parkdrone_vision.sim_uplink <area>` | `sim_uplink.py` | **live sidecar**: watches `sim/output/<area>/` and POSTs frames as a Webots flight writes them, so the map fills in mid-patrol. Reads only — the flight controller stays free of network code, and disk remains the source of truth for its resume logic |
| `python -m parkdrone_vision.cleanup` | `cleanup.py` | one frame-retention sweep by hand (the server also runs it every `CLEANUP_INTERVAL_S`); exits 1 if it found expired frames still queued |

Both ingest tools share `ingest_client.py` — one implementation of the multipart body, since that
is the wire format real drone firmware will have to reproduce.

The whole stack — infra, migrations, seed, server, both UIs — comes up with
`pnpm quickstart` from `web/` (`scripts/quickstart.sh`).

## What each module does

### `api/` — the HTTP and WebSocket edge

| Module | Responsibility |
| --- | --- |
| `app.py` | The FastAPI app and every route. **Ingest**: `POST /api/v1/ingest/mission/start`, `…/{id}/end`, `…/frame`. **Reads**: `GET /api/v1/bays` (GeoJSON, optional `bbox`/`zona`), `/summary`, `/bays/{id}`. **Directions**: `GET /api/v1/route`. **Metrics**: `GET /api/v1/metrics` (JSON, optional `window_s`), `GET /metrics` (Prometheus). **Dev toggle**: `POST /api/v1/dev/occupy\|free` (mounted unless `ENABLE_DEV_ROUTES=false`). **Realtime**: `WS /ws/occupancy`. Its `lifespan` opens the pool, loads bays as ENU rings, runs crash recovery, then starts the classify threads. Also normalises the pose's `frame_idx` once, at the edge, so nothing downstream needs a legacy-`i` fallback. |
| `auth.py` | `require_drone` FastAPI dependency: reads `x-api-key`, matches its SHA-256 against `drone.api_key_hash`, bumps `last_seen`, returns `drone_id`. 401 missing / 403 unknown. |
| `hub.py` | WebSocket fan-out. `broadcast(deltas)` tags each as `{type:"bay_delta", …}` and sends to every client, dropping dead sockets. A 500-entry replay deque plus a monotonic cursor lets a reconnecting client resume with `?since=`. |
| `metrics.py` | Operational snapshot for the admin dashboard and any scraper. Two halves: **in-process** (`jobs.stats()` — queue depth, in-flight, process-lifetime totals, mean classify time; these exist nowhere else and reset with the process) and **durable** (`web_db` — job status counts, oldest-queued age, ingest rates, enqueue→finish latency percentiles, fleet/mission progress, bay coverage, and model accuracy where ground truth exists — `state_accuracy` after the vote, `view_accuracy` before it, both `null` rather than `0` without labels). `prometheus(snap)` renders the numeric subset as text exposition — no client library; windowed DB figures are exported as gauges, only the lifetime process totals as counters. |

### `processing/` — frame → occupancy

| Module | Responsibility |
| --- | --- |
| `jobs.py` | The job queue: a `queue.Queue` drained by `CLASSIFY_THREADS` daemon threads, each owning its **own** long-lived psycopg2 connection (connections aren't thread-shareable). Each job: fetch the image → `process_frame` → mark the job processed → schedule `hub.broadcast` on the event loop. A missing-image `ClientError` marks the job `failed` so recovery won't loop on it; any other exception is logged and the thread survives. `recover(conn)` rebuilds the queue from `frame_job.status='queued'` on boot. `stats()` exposes the live counters (depth, in-flight, processed/failed/recovered/deltas, mean classify time) that `api/metrics.py` reports. |
| `pipeline.py` | The whole scoring transaction: resolve eval labels → `score_frame` → `insert_observations` → `recompute_states` → commit → return deltas. Shared verbatim by the live threads and the offline `replay.py`, which is what makes the golden test meaningful. Labels are looked up here rather than in each caller (`ground_truth.labels_for`, overridable with an explicit `gt=`) because they are a property of the survey area — so live ingest and replay both record them and accuracy is measured on the production path. |

### `vision/` — the classifier bridge

| Module | Responsibility |
| --- | --- |
| `scoring.py` | Puts `<repo>/vision` on `sys.path` and imports `score_occupancy` (side-effect free — its `main()` is guarded). Re-exports `project`, `bay_features`, `classify`, `to_enu`, `pose_idx` so the web tier **never re-implements the calibrated thresholds**. Adds `score_frame(img, bays, pose)`: for each bay, project its ring to pixels, skip if the bbox misses the frame or the core isn't visible enough, else classify and record the centre offset. |
| `ground_truth.py` | **Eval only.** `labels_for(survey_area)` → `{bay_id: occupied}` read from `sim/worlds/<area>.ground_truth.json` (plus the legacy bare `ground_truth.json` for `fmi_block`), which is what makes `observation.gt` — and therefore the accuracy figures on `/metrics` — non-empty. Caches hits **and misses**, since production is the miss case and must not re-stat the filesystem once per frame. The survey-area name arrives from the drone over HTTP and is about to become a file path, so it is whitelisted (`^[A-Za-z0-9_-]{1,64}$`), not escaped. No file → `gt = NULL` → accuracy reported as unknown, which is the correct production answer. |

### `db/` — all SQL in one place

| Module | Responsibility |
| --- | --- |
| `pool.py` | `ThreadedConnectionPool` (1–10) + a `borrow(commit=False)` context manager. Used by the sync request handlers, which Starlette runs in its threadpool — blocking psycopg2 never touches the event loop. Classify threads deliberately bypass this pool. |
| `web_db.py` | Web-edge SQL. Reads (`feature_collection`, `summary`, `detail`, `nearest_bay`) push geospatial predicates into PostGIS. Writes: `insert_frame` (idempotent on `(drone_id, survey_area, frame_idx)`, and creates the `frame_job` row in the *same* transaction), mission bookkeeping, `mark_frame_processed/failed`, `unscored_frames` (the recovery backlog), plus the auth lookups. Metrics queries (`job_counts`, `ingest_rates`, `classify_latency`, `fleet_health`, `coverage_counts`) are bounded scans over the retention-swept tables, riding the existing `frame_received_idx` / `frame_job_status_idx`. |
| `vision_db.py` | Vision-side SQL: `load_bays_enu` (WGS84 → ENU rings), `insert_observations` (one row per scored bay, numpy scalars coerced to float), and `recompute_states` — strict-majority vote over a bay's observations, upsert `bay_state`, and return a delta only for bays that flipped or became known. |

### Package root — config and adapters

| Module | Responsibility |
| --- | --- |
| `config.py` | Walks up from CWD for the nearest `.env` (process env always wins) and exposes `DATABASE_URL`, the S3 settings, `SIM_OUTPUT_ROOT`, `API_PORT`, `ENABLE_DEV_ROUTES`, `CLASSIFY_THREADS`, `GROUND_TRUTH_ROOT`, and the OSRM/cache knobs. |
| `s3.py` | The single boto3 client both sides share: `put_frame` (ingest writes bytes, stores only the `s3://` uri on the frame row) and `get_frame_array` (classify threads read back an HxWx3 RGB array; also accepts a local path, which is how replay works). |
| `routing.py` | Proxies driving directions to OSRM so the driver's coordinates stay on our origin and the provider is swappable. Quantised-coordinate TTL cache absorbs the map's re-routes; `urllib` only (no new dependency), with a one-shot unverified retry for Windows' flaky cert revocation. Raises `RoutingError` → 502, and the map falls back to a straight line. |

## Why it is split this way

- **`api/` vs `processing/` is the thread boundary, not just a folder.** Everything in `api/`
  runs on request threads (or the event loop, for the WS endpoint); everything in
  `processing/` runs on the classify threads. `jobs.enqueue` and
  `asyncio.run_coroutine_threadsafe(hub.broadcast(...))` are the only two crossings.
- **`web_db.py` vs `vision_db.py` mirrors that same split**, which is why they don't share a
  connection strategy: web SQL borrows from `pool.py` per request, vision SQL uses one
  dedicated connection per classify thread.
- **`vision/scoring.py` exists so the classifier has exactly one home.** The tuned thresholds
  live in `vision/score_occupancy.py`; the server imports them rather than copying them, and
  `replay.py` proves the live path still agrees with the offline one.
- **`frame` (ledger) and `frame_job` (work state) are written in one transaction**, which is
  what lets an in-memory queue be durable: on restart `jobs.recover` rebuilds it from
  Postgres + S3. See `docs/db_schema_er.md`.
