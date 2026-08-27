-- Parking as a 1-D resource along a kerb, beside the per-bay model.
--
-- The per-bay rule is measurably not good enough on real footage. Hand-counted
-- ground truth for flight 0035 (18 segments, ~280 m of kerb, 29 cars) says
-- `bay_votes_from_dets` finds 9 of them -- 31% -- and its under-counting is
-- statistically certain (bias -1.11 cars/segment, 95% CI [-1.50, -0.78]). The
-- live map therefore shows that street as 10 occupied of 88, ~11%, when it is
-- about 57% full. That is the "everything is free" failure, measured.
--
-- The cause is not detection: after the BGR fix the detector finds 599 cars on
-- that flight and 473 of them (79%) are attributed to NO bay. It is that
-- Sofiaplan's rectangles are wrong by more than a bay width and no rigid
-- correction fixes them -- global shift leaves 3.59 m, per-street 3.35 m,
-- per-row-side 2.84 m, against 3.99 m raw. The error is anisotropic (cross-street
-- 3.14 m vs along-street 1.64 m), and a gap length along a kerb is invariant to
-- sliding every car on it by the same amount while a per-bay boolean is not.
--
-- So a `curb_run` is a polyline with an arclength, a capacity and a pitch, built
-- from the bays by geometry (tools/make_runs.py) -- never by street name, the
-- same rule street_closure follows, and for the sharper reason that one street's
-- two sides need opposite-signed corrections.
--
-- **Nothing here replaces the bay tables.** `bay`, `observation` and `bay_state`
-- are untouched and the per-bay path still runs on every frame. The two answers
-- are meant to be served side by side until the run layer has earned the swap:
-- the sim, the golden fixtures and every accuracy figure on record are per-bay,
-- and dropping that would make the two incomparable.

-- The runs themselves: reference data, seeded from data/curb_runs.geojson ------
CREATE TABLE IF NOT EXISTS curb_run (
  run_id       text PRIMARY KEY,
  street       text,
  side         text,
  zona         text,
  park_txt     text,
  -- Sofiaplan's own count of spaces on this stretch, and the ONE thing in that
  -- dataset this design still trusts: every coordinate is distrusted, but how
  -- many spaces a kerb holds is what the city knows and the camera cannot see.
  -- Never recompute it as length/pitch -- published pitch is 5.41 m where the
  -- real one is 4.48 m, which would undercount a 35 m run by about a space.
  capacity     integer NOT NULL,
  pitch_m      real,
  length_m     real,
  -- false for a chain of <4 bays: too few points to establish an axis. Kept
  -- rather than dropped, so "we do not know" stays distinct from "not there".
  verified     boolean NOT NULL DEFAULT true,
  bay_ids      text[] NOT NULL DEFAULT '{}',
  geom         geometry(LineString, 4326) NOT NULL
);
CREATE INDEX IF NOT EXISTS curb_run_geom_idx ON curb_run USING GIST (geom);

-- Detections placed on a run, as points on the ground ------------------------
-- Points and not intervals, because cars are counted as INSTANCES: clustering
-- the same car's many views into one is what makes the count unbiased, and that
-- needs the raw positions. Deriving a count from occupied LENGTH was the first
-- version and it under-counts with certainty (bias -0.53, CI [-0.91, -0.15]) --
-- it loses a car when a cell fails the majority vote, and again when two
-- adjacent cars merge into one interval.
--
-- No FK to `frame`, deliberately: frames are transient (FRAME_RETENTION_S, 4 h)
-- and these are the record, exactly like `observation`. `world` and
-- `observed_at` are carried so a row stands on its own once its frame is gone.
CREATE TABLE IF NOT EXISTS run_detection (
  id           bigserial PRIMARY KEY,
  run_id       text NOT NULL REFERENCES curb_run(run_id),
  world        text NOT NULL,
  mission_id   text,
  frame_idx    integer NOT NULL,
  -- text, matching frame.frame_id and observation.frame_id -- it is a uuid
  -- string, not a serial
  frame_id     text,
  s            real NOT NULL,          -- arclength along the run, metres
  x            double precision NOT NULL,
  y            double precision NOT NULL,
  conf         real,
  observed_at  timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS run_detection_run_time_idx
  ON run_detection(run_id, observed_at DESC);
CREATE INDEX IF NOT EXISTS run_detection_frame_idx ON run_detection(frame_id);

-- Which stretch of a run each frame actually LOOKED at ------------------------
-- Without this there is no way to tell empty kerb from unobserved kerb, and
-- reporting the second as free parking is the one failure worse than saying
-- nothing. A run can leave and re-enter a frame, so a frame may write several
-- spans for one run.
CREATE TABLE IF NOT EXISTS run_observation (
  id           bigserial PRIMARY KEY,
  run_id       text NOT NULL REFERENCES curb_run(run_id),
  world        text NOT NULL,
  mission_id   text,
  frame_idx    integer NOT NULL,
  -- text, matching frame.frame_id and observation.frame_id -- it is a uuid
  -- string, not a serial
  frame_id     text,
  s0           real NOT NULL,
  s1           real NOT NULL,
  observed_at  timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS run_observation_run_time_idx
  ON run_observation(run_id, observed_at DESC);
CREATE INDEX IF NOT EXISTS run_observation_frame_idx ON run_observation(frame_id);

-- The published answer, one row per run --------------------------------------
-- `capacity_observed` is capacity scaled to the stretch actually seen: a flight
-- that covered a third of a run cannot speak for the rest, and reporting the
-- full capacity would invent free spaces out of kerb nobody looked at. `free` is
-- derived from it, never from `capacity`.
CREATE TABLE IF NOT EXISTS run_state (
  run_id             text PRIMARY KEY REFERENCES curb_run(run_id),
  world              text NOT NULL,
  mission_id         text,
  cars               integer NOT NULL,
  free               integer NOT NULL,
  capacity_observed  integer NOT NULL,
  observed_fraction  real NOT NULL,
  backend            text,
  updated_at         timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX IF NOT EXISTS run_state_world_idx ON run_state(world);
