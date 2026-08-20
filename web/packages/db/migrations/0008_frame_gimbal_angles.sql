-- Store the gimbal joint angles alongside the body attitude.
--
-- `project()` stopped assuming the camera is nadir (2026-08-20): the gimbal's
-- roll joint sits below its pitch joint, so at the +pi/2 pitch this survey
-- flies it spins the image about the optical axis instead of levelling the
-- camera. Reconstructing where a bay landed in a frame therefore needs
-- cam_pitch and cam_roll, not just roll/pitch.
--
-- Without these columns the live path and the restart-recovery path would
-- disagree: recover() rebuilds a job's pose from `frame`, so a recovered frame
-- would be projected with a nadir gimbal and could classify differently from
-- the same frame processed live. Nullable, because frames ingested before this
-- (and any drone that does not report a gimbal) simply have no value -- the
-- projection falls back to nadir for those, exactly as it did then.
ALTER TABLE frame ADD COLUMN IF NOT EXISTS cam_pitch double precision;
ALTER TABLE frame ADD COLUMN IF NOT EXISTS cam_roll  double precision;
