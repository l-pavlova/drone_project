# PARKDRONE Web Infrastructure

Turns the drone's on-disk per-bay occupancy report into a live product: an ingest+processing
API, real-time occupancy push, an end-user parking map, and an admin analytics dashboard.

See the architecture & requirements in `docs/web_infra_plan.md`, and the server's internal
module map in `docs/server_modules.md`.

## Layout (pnpm monorepo)

```
web/
  packages/
    contracts/   # shared TS types + zod schemas (pose / bay / delta) + ENU projection
    db/          # Postgres+PostGIS migrations, geojson seeder
  apps/
    vision-worker/  # the whole server: FastAPI edge + in-process CV     (phases 2-4)
    web-user/    # React + react-leaflet end-user dashboard               (phase 5)
    web-admin/   # React admin analytics                                  (phase 6)
  infra/         # docker-compose (postgis + minio), k8s                  (phase 7)
```

## Getting started (dev)

```bash
cd web
cp -n .env.example .env   # required: compose reads its credentials from here
pnpm install
pnpm infra:up          # postgis + minio via docker compose
pnpm db:migrate        # create schema (enables PostGIS, GiST index on bay.geom)
pnpm db:seed           # load data/block_bays.geojson into the bay table
```

### Re-flying a survey area

```bash
pnpm clear                      # list what is stored, delete nothing
pnpm clear fmi_block            # DB rows + stored images + sim/output captures
pnpm clear fmi_block --db-only  # keep the captures on disk (--disk-only is the mirror)
pnpm quickstart --clear --fly fmi_block_4st    # clear, then fly it, in one command
```

Ingest is idempotent on `(drone_id, survey_area, frame_idx)`, so flying an area the stack has
already seen posts duplicates: no classify job, no `bay_state` change, and — since deltas fire
only on a *change* — nothing on the map, for a whole patrol, with no error anywhere. Clearing the
area first is what makes a re-flight count. It also removes `sim/output/<area>/poses.json`, which
the controller would otherwise *resume* from instead of re-flying, and it refuses a folder holding
`occupancy_results.json` (a scored golden fixture) unless you pass `--force`.

Needing this at all is a known wart — see TODO #10: a re-flight should ingest itself, keyed per
mission rather than per survey area.

The persistence layer is **Postgres + PostGIS from the start** (no SQLite phase): a drone fleet
writes concurrently, and PostGIS gives native spatial indexing so bbox / nearest-bay queries run
in the database. All bay geometry stays in WGS84 (EPSG:4326); the shared ENU projection
(`packages/contracts/src/geo.ts`) mirrors `sim/generate_world.py` / `vision/score_occupancy.py`
and is used for pose math only.
