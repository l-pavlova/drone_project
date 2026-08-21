-- Re-key ingest idempotency from the survey area to the MISSION.
--
-- The old key was UNIQUE (drone_id, survey_area, frame_idx), and `frame_idx`
-- restarts at 0 on every flight. So a second flight of an area already in the
-- database posted frames that came back 200-duplicate: no classify job, no
-- bay_state change, and — because deltas only fire on a change — nothing on the
-- map, for a whole patrol, with no error anywhere. The only way to re-survey was
-- to DELETE the history first (`pnpm clear`), which is backwards: wiping the
-- analytics record in order to record something new, and impossible in a real
-- deployment where `observation` is the product's history.
--
-- A mission already means "one flight", so keying on it says what was actually
-- meant: the same frame re-sent within a flight is still a duplicate (the retry
-- case the constraint was written for), while a NEW flight is new data.
--
-- mission_id is nullable and NULLs are distinct in a unique index, so a
-- mission-less frame would lose deduplication entirely. Hence the COALESCE
-- sentinel: without a mission, the key falls back to exactly the old
-- (drone, area, frame_idx) behaviour, so older clients keep the semantics they
-- were written against instead of silently getting none.
ALTER TABLE frame DROP CONSTRAINT IF EXISTS frame_drone_id_survey_area_frame_idx_key;
CREATE UNIQUE INDEX IF NOT EXISTS frame_drone_mission_frame_idx_key
    ON frame (drone_id, COALESCE(mission_id, 'area:' || survey_area), frame_idx);

-- The vote needs to know which flight an observation came from, so that two
-- flights inside one OCCUPANCY_WINDOW_S do not average a stale look together
-- with a fresh one: the newest mission that saw a bay wins it outright, and the
-- earlier observations stay as history rather than as votes.
ALTER TABLE observation ADD COLUMN IF NOT EXISTS mission_id text;

-- Best-effort backfill for rows already on disk: only where a (survey_area,
-- frame_idx) pair maps to exactly ONE mission, since anything else is a guess.
-- Rows left NULL are handled by the vote's IS NOT DISTINCT FROM, which groups
-- all mission-less observations of a bay together exactly as before.
UPDATE observation o
   SET mission_id = f.mission_id
  FROM (SELECT survey_area, frame_idx, min(mission_id) AS mission_id
          FROM frame
         WHERE mission_id IS NOT NULL
         GROUP BY survey_area, frame_idx
        HAVING count(DISTINCT mission_id) = 1) f
 WHERE o.mission_id IS NULL
   AND o.survey_area = f.survey_area
   AND o.frame_idx = f.frame_idx;

-- Supports the per-bay "which mission saw this last" lookup the vote opens with.
CREATE INDEX IF NOT EXISTS observation_bay_mission_idx
    ON observation (bay_id, survey_area, observed_at DESC, mission_id);
