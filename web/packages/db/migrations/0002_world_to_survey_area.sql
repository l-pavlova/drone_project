-- Rename `world` -> `survey_area` on every table that carries it.
--
-- "world" was Webots vocabulary that leaked out of the simulator into the
-- product schema. What the column actually identifies is the patch of city a
-- patrol covers ("fmi_block"), so it is named for that. Values are unchanged;
-- only the column name moves.
--
-- Forward-only: 0001 is left as the historical record, since databases that
-- already ran it are skipped by the schema_migrations ledger.

ALTER TABLE mission     RENAME COLUMN world TO survey_area;
ALTER TABLE observation RENAME COLUMN world TO survey_area;
ALTER TABLE frame       RENAME COLUMN world TO survey_area;

-- Indexes and constraints follow the column automatically, but their NAMES
-- keep the old word; rename them too so \d output stays readable.
ALTER INDEX observation_world_idx RENAME TO observation_survey_area_idx;

-- The idempotent-ingest unique constraint UNIQUE (drone_id, world, i) was
-- auto-named by Postgres at creation.
ALTER TABLE frame RENAME CONSTRAINT frame_drone_id_world_i_key
                               TO frame_drone_id_survey_area_i_key;
