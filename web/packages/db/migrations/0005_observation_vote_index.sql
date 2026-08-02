-- Index for the windowed occupancy vote. No schema or data changes.
--
-- The vote is now "majority over the observations of this bay, in this survey
-- area, inside OCCUPANCY_WINDOW_S", and it runs once per classified frame.
-- observation_bay_time_idx (bay_id, observed_at DESC) already exists but has no
-- survey_area, so every candidate row had to be fetched and rechecked. With
-- survey_area in the key the aggregate is a bounded index range scan, and its
-- cost stops depending on how much history the bay has accumulated.
CREATE INDEX IF NOT EXISTS observation_bay_area_time_idx
    ON observation (bay_id, survey_area, observed_at DESC);
