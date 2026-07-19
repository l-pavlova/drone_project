-- PARKDRONE web schema (Postgres + PostGIS).
-- Idempotent-ish: guarded by the migrate runner (schema_migrations ledger).

CREATE EXTENSION IF NOT EXISTS postgis;

-- Fleet registry -----------------------------------------------------------
CREATE TABLE drone (
  drone_id      text PRIMARY KEY,
  name          text NOT NULL,
  api_key_hash  text NOT NULL,
  created_at    timestamptz NOT NULL DEFAULT now(),
  last_seen     timestamptz
);

CREATE TABLE mission (
  mission_id       text PRIMARY KEY,
  drone_id         text NOT NULL REFERENCES drone(drone_id),
  world            text NOT NULL,
  area             text,
  started_at       timestamptz NOT NULL DEFAULT now(),
  ended_at         timestamptz,
  frames_expected  integer,
  frames_done      integer NOT NULL DEFAULT 0
);
CREATE INDEX mission_drone_idx ON mission(drone_id, started_at DESC);

-- Static bay geometry (seeded from data/block_bays.geojson) -----------------
CREATE TABLE bay (
  bay_id       text PRIMARY KEY,
  zona         text,
  street       text,
  park_txt     text,
  bearing_deg  double precision,
  public       boolean NOT NULL DEFAULT false,
  geom         geometry(Polygon, 4326) NOT NULL,
  centroid     geometry(Point, 4326) NOT NULL
);
-- Spatial index powers bbox filtering and nearest-bay ("free spaces near me").
CREATE INDEX bay_geom_gix ON bay USING GIST (geom);
CREATE INDEX bay_centroid_gix ON bay USING GIST (centroid);
CREATE INDEX bay_zona_idx ON bay(zona);

-- Current occupancy: one row per bay, upserted by the vision worker ---------
CREATE TABLE bay_state (
  bay_id      text PRIMARY KEY REFERENCES bay(bay_id),
  occupied    boolean NOT NULL,
  confidence  real NOT NULL DEFAULT 0,
  last_frame  integer,
  updated_at  timestamptz NOT NULL DEFAULT now(),
  source      text NOT NULL DEFAULT 'vision' CHECK (source IN ('vision', 'manual'))
);
CREATE INDEX bay_state_updated_idx ON bay_state(updated_at DESC);

-- Append-only classification history (feeds admin analytics) ----------------
CREATE TABLE observation (
  id               bigserial PRIMARY KEY,
  bay_id           text NOT NULL REFERENCES bay(bay_id),
  occupied         boolean NOT NULL,
  frame_idx        integer NOT NULL,
  world            text NOT NULL,
  votes_occupied   integer NOT NULL DEFAULT 0,
  views            integer NOT NULL DEFAULT 0,
  vis              real,
  center_off_px    real,
  core_paint_frac  real,
  core_dark_frac   real,
  core_chroma      real,
  core_brightness  real,
  core_std         real,
  gt               boolean,
  observed_at      timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX observation_bay_time_idx ON observation(bay_id, observed_at DESC);
CREATE INDEX observation_world_idx ON observation(world, observed_at DESC);

-- Ingest ledger: one row per uploaded frame ---------------------------------
CREATE TABLE frame (
  frame_id     text PRIMARY KEY,
  drone_id     text NOT NULL REFERENCES drone(drone_id),
  mission_id   text REFERENCES mission(mission_id),
  world        text NOT NULL,
  i            integer NOT NULL,
  x            double precision NOT NULL,
  y            double precision NOT NULL,
  alt          double precision NOT NULL,
  yaw          double precision NOT NULL,
  roll         double precision,
  pitch        double precision,
  image_uri    text NOT NULL,
  status       text NOT NULL DEFAULT 'queued'
                 CHECK (status IN ('queued', 'processed', 'failed')),
  received_at  timestamptz NOT NULL DEFAULT now(),
  processed_at timestamptz,
  -- one frame index per drone per world (idempotent ingest)
  UNIQUE (drone_id, world, i)
);
CREATE INDEX frame_status_idx ON frame(status, received_at);
CREATE INDEX frame_mission_idx ON frame(mission_id);
