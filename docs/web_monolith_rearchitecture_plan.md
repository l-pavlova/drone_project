# Plan — Collapse the web tier into a Python FastAPI monolith (drop Redis)

## Context

Today the `web/` runtime is three processes: a **Node/Express API** (`apps/api`), a
**separate Python vision worker** (`apps/vision-worker`), and **Redis** bridging them
(a `parkdrone:jobs` list + a `parkdrone:deltas` pub/sub channel). The process split
and Redis exist for exactly one reason: the classifier is **Python**
(`score_occupancy.py`, reused verbatim) and the web edge is **Node** — two languages
can't share a process, so frames must cross a durable queue and deltas must be relayed
back for the WebSocket.

If the web edge is **also Python**, that constraint disappears. The classifier can be
`import`ed and called in-process (sim-parity preserved), the job queue becomes an
in-process `queue.Queue`, and deltas are pushed straight to the WebSocket clients the
same process already holds — **Redis is deleted entirely, and the separate worker pod
goes away.** This plan replaces `apps/api` + `apps/vision-worker` with a single FastAPI
service, keeping the React app and the operator scripts working against an unchanged
HTTP/WS contract.

**Non-goal:** changing any classification behaviour. `vision_core.py` and
`pipeline.process_frame` are reused **untouched** — the golden replay test must still
pass 43/43.

## Target architecture

One `uvicorn` process:

```
POST /ingest/frame ─┐         (sync def → Starlette threadpool: psycopg2 + boto3)
                    ├─ persist frame (S3 put + frame INSERT, idempotent) → 202
                    └─ jobs.put(job)                    in-process queue.Queue
                                        │
              ┌─────────────────────────┴───────────── N classify threads ─────┐
              │  each owns its own psycopg2 conn + the shared ENU bays list      │
              │  job → pipeline.process_frame(...) (classify → observations →    │
              │        recompute_states → deltas)                                │
              │  deltas → asyncio.run_coroutine_threadsafe(hub.broadcast, loop)  │
              └─────────────────────────┬───────────────────────────────────────┘
                                        ▼
                             WS /ws/occupancy hub  → browsers
GET /bays·/summary·/bays/:id ── sync def → PostGIS (same SQL as repositories.ts)
```

**Concurrency model (the crux — get this right):**
- **Event loop stays free.** Read/ingest handlers are declared **sync `def`**, so
  Starlette runs them in its anyio threadpool; blocking `psycopg2`/`boto3` never touch
  the loop. Only the WebSocket endpoint is `async`.
- **CV runs on a dedicated thread pool**, *separate* from the request threadpool — a
  fixed set of N OS threads draining a `queue.Queue`. `numpy` releases the GIL during
  array ops, so classify threads get real parallelism and never stall ingest.
- **Thread → loop bridge for deltas:** capture the running loop at startup; classify
  threads publish via `asyncio.run_coroutine_threadsafe(hub.broadcast(payload), loop)`.
- **Durability replacement:** an in-memory queue loses in-flight jobs on restart. Frames
  are already persisted (S3 + `frame` row), so on startup we **re-enqueue unscored
  frames** (see migration below). This is strictly cleaner than trusting the broker.

## What is reused verbatim (no edits)

- `apps/vision-worker/parkdrone_vision/vision_core.py` — `score_frame`, `to_enu` (the classifier + ENU projection).
- `.../pipeline.py` — `process_frame` already returns `{"scored","deltas"}`; call it with
  `redis_client=None` and broadcast the returned `deltas` ourselves. **No change.**
- `.../db.py` — `load_bays_enu`, `insert_observations`, `recompute_states`.
- `packages/db/migrations/*.sql` + `seed.ts` stay as the schema/seeder tooling (run via
  `pnpm db:migrate`/`db:seed`); they are not on the runtime path.
- `packages/contracts` (TS) stays — the React app still imports its types; it remains the
  canonical ORIGIN/MLAT/MLON spec that `vision_core.to_enu` already mirrors.

## New/changed files

Consolidate the runtime into one Python package (promote `apps/vision-worker` →
`apps/server`, or add these modules beside the existing `parkdrone_vision` package):

- `app.py` — FastAPI app + `lifespan`: open a `psycopg2.pool.ThreadedConnectionPool`,
  `db.load_bays_enu` once into a shared list, capture the event loop, start N classify
  threads, run startup crash-recovery re-enqueue, mount routers.
- `auth.py` — `hash_api_key` (sha256) + a `require_drone` FastAPI dependency: read
  `x-api-key`, look up `drone.api_key_hash`, bump `last_seen`, return `drone_id`
  (401/403 on miss). Mirrors `apps/api/src/auth.ts`.
- `routes_ingest.py` — `POST /api/v1/ingest/mission/start`, `/mission/{id}/end`,
  `/frame` (multipart `frame` + `meta`). Mirrors `ingest.ts`: S3 `put_frame`, idempotent
  `INSERT … ON CONFLICT (drone_id,world,i) DO NOTHING`, `mission.frames_done` bump,
  `jobs.put(...)`, `202`.
- `routes_read.py` — `GET /api/v1/bays` (bbox, zona), `/summary`, `/bays/{id}`. Port the
  **exact** PostGIS SQL from `packages/db/src/repositories.ts` (the `json_build_object`
  FeatureCollection, the zone summary FILTER counts, the detail+history query) so
  responses are byte-identical for the React app.
- `routes_dev.py` — `POST /api/v1/dev/occupy|free` (guarded by `ENABLE_DEV_ROUTES`).
  Mirrors `dev.ts`: nearest-bay `<->` query, `bay_state` upsert with `source='manual'`,
  then `hub.broadcast`.
- `s3.py` — boto3 client + `put_frame(key, bytes)` and `get_frame_bytes(uri)` (lift the
  existing `_s3_client`/`load_image` logic out of `worker.py`).
- `hub.py` — WebSocket connection registry: `connect/disconnect`, `async broadcast(delta)`,
  bounded in-memory replay `deque` + cursor for `?since=` and optional `?bbox=` filtering
  and the `snapshot_cursor` message (parity with `ws.ts`).
- `jobs.py` — the `queue.Queue`, the classify-thread pool (`process_frame` per job then
  bridge deltas to `hub`), and `recover_unscored(conn)` for startup re-enqueue.
- `ws.py` — `@app.websocket("/ws/occupancy")` registering the socket with `hub`.
- Port operator scripts to Python (since `apps/api` is removed): `register_drone.py`
  (insert `drone` + print raw key) and `replay_ingest.py` (HTTP client posting
  `poses.json` frames with `x-api-key`) — direct ports of the two `apps/api/src/scripts/*.ts`.

**Delta shape (must match the frontend):** `hub.broadcast` emits
`{type:"bay_delta", bay_id, occupied, confidence, updated_at}` — exactly what
`useOccupancySocket.ts` consumes and what `process_frame` already returns per delta.

### One tiny migration (crash-recovery precision)

Add `frame.scored_at timestamptz` (new `packages/db/migrations/0002_scored_at.sql`).
`process_frame` sets it when a frame finishes; startup re-enqueues
`SELECT … FROM frame WHERE scored_at IS NULL`. This also removes today's
"observations are additive on re-process" double-count risk — reprocessing becomes a
clean no-op instead of "harmless but stable".

## What gets removed

- `apps/api` (entire Node service) and `apps/vision-worker/parkdrone_vision/worker.py`
  (the standalone BRPOP loop; its S3/image logic moves to `s3.py`).
- `apps/api/src/redis.ts`, the `ioredis` dependency, and the `redisUrl/jobsQueue/deltasChannel` config.
- `packages/db/src/repositories.ts` (its SQL moves into `routes_read.py`; the seeder
  doesn't use it).
- The **`redis` service** in `infra/docker-compose.yml` (leaves `postgres` + `minio` +
  `minio-init`). Update root `package.json` scripts + `CLAUDE.md` run instructions.

## New dependencies

`fastapi`, `uvicorn[standard]` (brings the `websockets` impl), `python-multipart`
(form parsing). `numpy`, `Pillow`, `psycopg2`, `boto3` already present.

## Keep unchanged

- **React app** (`apps/web-user`) — same `/api/v1` + `/ws/occupancy` contract; the vite
  proxy still targets `:4000`, so **run the FastAPI service on `API_PORT=4000`**.
- **Schema, migrations, seed** — still `pnpm db:migrate && pnpm db:seed`.
- **The 1-day frame-retention lifecycle rule** added earlier stays as-is.

## Verification (end to end)

1. **Classifier untouched:** `cd apps/vision-worker && python -m parkdrone_vision.replay fmi_block` → still **43/43 match, 100% vs GT** (proves `pipeline`/`vision_core` unchanged).
2. **Boot monolith:** `uvicorn app:app --port 4000` — logs bays loaded + N classify threads + "recovered K unscored frames".
3. **No Redis anywhere:** `docker compose … config` shows no redis; `grep -ri redis apps/ packages/` is clean.
4. **Full E2E:** `python -m parkdrone_service.register_drone drone-1` → key; then
   `API_KEY=<key> python -m parkdrone_service.replay_ingest fmi_block` → frames accepted
   (202), WS deltas observed, final `GET /api/v1/bays` occupancy matches
   `occupancy_results.json` (same assertion the TS `replay-ingest` made).
5. **Live dashboard:** vite on `:5173` (proxy unchanged) → `curl -X POST :4000/api/v1/dev/occupy` flips the bay red/green over the socket.
6. **Restart durability:** kill mid-replay, restart — startup re-enqueue finishes the
   unscored frames; final `/bays` is identical to the uninterrupted run.

## Migration order (low-risk sequence)

1. Add `0002_scored_at.sql`; migrate. 2. Build the Python package (reuse `db.py`/
`pipeline.py`/`vision_core.py`; add `jobs.py`, `hub.py`, `s3.py`, routes, `app.py`).
3. Stand it up on `:4000` and pass verification 1–6 **while Node is still present**.
4. Only then delete `apps/api`, `worker.py`, `repositories.ts`, the redis service/dep,
   and update `package.json` + `CLAUDE.md`.
