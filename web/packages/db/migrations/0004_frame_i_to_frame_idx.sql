-- Rename `frame.i` -> `frame.frame_idx`.
--
-- The same number travelled the tier under three names: `i` in poses.json and
-- in this table, `frame_idx` in observation / the zod contracts / the API
-- responses. `frame_idx` wins because it is the one that already reads clearly
-- at every other site; a bare `i` is fine as a loop variable and poor as a
-- column, where it shows up in raw SQL with no context (`f.i`, and a constraint
-- literally named ..._i_key).
--
-- Values are unchanged; only the name moves. The pose JSON key is renamed in
-- lockstep (parkdrone.py now writes `frame_idx`), with readers still accepting
-- legacy `i` so poses.json already on disk keeps replaying — see
-- vision/score_occupancy.py:pose_idx.
--
-- Forward-only, per 0002: databases that already ran the earlier migrations are
-- skipped by the schema_migrations ledger, so 0001/0003 stay as written.

ALTER TABLE frame RENAME COLUMN i TO frame_idx;

-- The idempotent-ingest unique constraint UNIQUE (drone_id, survey_area, i) was
-- auto-named by Postgres at creation and renamed once already in 0002; its name
-- still carries the old column, so move it too and keep \d output readable.
ALTER TABLE frame RENAME CONSTRAINT frame_drone_id_survey_area_i_key
                               TO frame_drone_id_survey_area_frame_idx_key;
