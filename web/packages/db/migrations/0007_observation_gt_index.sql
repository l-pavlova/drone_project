-- Partial index for the model-accuracy readings. No schema or data changes.
--
-- Accuracy is only defined where ground truth exists — sim and eval runs — and
-- those rows are a small minority of `observation` (in production `gt` is always
-- NULL, because there is nothing to compare against). A plain scan would
-- therefore read the entire history on every dashboard poll to find the few
-- labelled rows, and `observation` is the table designed to grow forever.
--
-- The WHERE clause makes the index itself small: it holds only labelled
-- observations, so both accuracy queries — the per-view aggregate and the
-- DISTINCT ON (bay_id) latest-label lookup — become index-only work whose cost
-- scales with the eval set, not with the production history beside it.
CREATE INDEX IF NOT EXISTS observation_gt_idx
    ON observation (bay_id, observed_at DESC)
 WHERE gt IS NOT NULL;
