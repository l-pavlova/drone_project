-- Which MODEL decided a bay, recorded on the datum rather than inferred.
--
-- Two backends can now produce an occupancy verdict (config.OCCUPANCY_BACKEND):
-- the colour heuristic calibrated on Webots tones, which is the only thing that
-- reproduces the simulator numbers on record, and a fine-tuned whole-frame car
-- detector, which is the only thing that works on real photographs. They are
-- not interchangeable and their results must never be averaged, so "which one
-- was this?" has to survive into the database -- an observation row otherwise
-- carries five colour statistics that the detector path simply does not have,
-- and a NULL there would be indistinguishable from a failed measurement.
--
-- All columns are NULLable and nothing backfills: rows written before this
-- migration were all heuristic, but stamping them retroactively would be
-- asserting a fact about data this migration never saw. NULL reads as "not
-- recorded", which is true.
ALTER TABLE observation ADD COLUMN IF NOT EXISTS backend text;
ALTER TABLE observation ADD COLUMN IF NOT EXISTS det_score real;

-- bay_state carries the backend of the flight that currently owns the bay, so a
-- read path (and the map popup) can show provenance without joining back to the
-- observations the vote was taken over.
ALTER TABLE bay_state ADD COLUMN IF NOT EXISTS backend text;
