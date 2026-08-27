-- 0015: WHERE the free kerb is, not just how much of it there is.
--
-- `run_state.free` was `capacity_observed - cars` -- a subtraction, not a
-- measurement. A subtraction can advertise a space that does not physically
-- exist: four badly-spaced cars on a 30 m run leave it reporting 2 free while
-- the actual gaps are 1.5 m each and nothing fits. A driver feels that error
-- directly, and the subtraction cannot express WHERE to go at all.
--
-- `curb_runs.run_summary` now turns the counted car instances back into
-- intervals and measures the gaps between them (`free_gaps`), which yields both
-- a physically-realisable count and the gap positions. Measured on flight 0035:
-- 1102 spaces measured from gaps against 1164 by subtraction -- 62 spaces that
-- were being advertised and do not fit.
--
-- Both columns are NULLABLE with NO BACKFILL, the same posture as 0010's
-- `backend` and 0013's `unassigned_dets`. NULL means "not recorded", which is
-- the honest value for a row written before this migration; 0 would assert a
-- fact the migration never saw, and is indistinguishable from a real answer.
--
-- `gaps` is jsonb rather than a child table because a gap has no identity, no
-- history and no independent lifetime -- it is derived wholesale on every
-- recompute and is only ever read back with its run. `observation` earns a table
-- because it is the record; this is a rendering of the current answer.
ALTER TABLE run_state ADD COLUMN IF NOT EXISTS gaps jsonb;
ALTER TABLE run_state ADD COLUMN IF NOT EXISTS free_by_subtraction integer;

COMMENT ON COLUMN run_state.gaps IS
  'Measured free stretches: [{s0,s1,len_m,spaces}] in run arclength metres.';
COMMENT ON COLUMN run_state.free_by_subtraction IS
  'The pre-0015 estimate (capacity_observed - cars), kept beside the measured '
  'free so the two stay comparable rather than one silently replacing the other.';
