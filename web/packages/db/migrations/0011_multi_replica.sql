-- Make a second replica SAFE: job claiming, idempotent observations, and a
-- shared, globally ordered delta log.
--
-- Until now "one replica" was a correctness requirement rather than a
-- preference, for three reasons, and this migration is the schema half of
-- fixing all three:
--
--   1. Recovery re-enqueued every row with status='queued' and NO ownership
--      filter, so two replicas both drained the whole backlog: the same frame
--      classified twice, two observation rows for one look, and a majority vote
--      that double-counts.
--   2. The WebSocket hub was per-process, so a delta produced by replica A
--      never reached a browser held by replica B.
--   3. The replay cursor was a per-process counter starting at 0 every boot, so
--      ?since= meant something different on each replica (and after a restart).

-- ---------------------------------------------------------------------------
-- 1. frame_job gains a LEASE.
--
-- 'running' is a new legal state: claimed by a named replica, not yet finished.
-- A claim is a lease rather than a permanent assignment because the holder can
-- die; an expired lease is reclaimable by whoever notices, which is what turns
-- "recover my own work on restart" into "any live replica recovers a dead
-- one's work" -- strictly better availability, and the whole point of claiming.
--
-- `attempts` closes a separate, older hole: a job that failed for any reason
-- other than a missing image stayed 'queued' FOREVER and was re-run on every
-- single restart. With a counter it can be given up on.
ALTER TABLE frame_job
  ADD COLUMN IF NOT EXISTS claimed_by       text,
  ADD COLUMN IF NOT EXISTS claimed_at       timestamptz,
  ADD COLUMN IF NOT EXISTS lease_expires_at timestamptz,
  ADD COLUMN IF NOT EXISTS attempts         integer NOT NULL DEFAULT 0,
  ADD COLUMN IF NOT EXISTS last_error       text;

ALTER TABLE frame_job DROP CONSTRAINT IF EXISTS frame_job_status_check;
ALTER TABLE frame_job ADD  CONSTRAINT frame_job_status_check
  CHECK (status IN ('queued', 'running', 'processed', 'failed'));

-- Existing rows stay 'queued' with attempts = 0, i.e. simply claimable. No
-- backfill is needed or wanted.

-- The claim scan: unclaimed work, oldest first.
CREATE INDEX IF NOT EXISTS frame_job_claimable_idx
  ON frame_job (enqueued_at) WHERE status = 'queued';
-- The reaper scan, which is the same query's second arm: leases that lapsed.
CREATE INDEX IF NOT EXISTS frame_job_lease_idx
  ON frame_job (lease_expires_at) WHERE status = 'running';

-- ---------------------------------------------------------------------------
-- 2. observation becomes IDEMPOTENT per (frame, bay).
--
-- Claiming stops two replicas doing one frame at the same time. It does not
-- stop the same frame being done twice in SEQUENCE, and that window is real
-- even on a single replica: process_frame commits the observations, then
-- mark_frame_processed commits separately, so a crash in between leaves the job
-- claimable and the frame is scored again -- silently double-voting. A unique
-- key is the only place that can actually be guaranteed, so it goes here.
--
-- No FK: cleanup.py deletes frame rows at FRAME_RETENTION_S while observations
-- are kept as the analytics history, so an FK would either block the sweep or
-- null the column out from under the index.
ALTER TABLE observation ADD COLUMN IF NOT EXISTS frame_id text;

-- Partial, so pre-0011 rows (frame_id NULL) are excluded rather than colliding
-- with each other. Same posture as 0008/0010: never assert a fact about data
-- this migration did not see.
CREATE UNIQUE INDEX IF NOT EXISTS observation_frame_bay_key
  ON observation (frame_id, bay_id) WHERE frame_id IS NOT NULL;

-- ---------------------------------------------------------------------------
-- 3. bay_delta -- the shared, globally ordered delta log.
--
-- `id` is the cursor the WebSocket protocol hands clients. It has to come from
-- one sequence shared by every replica, or ?since= is meaningless the moment a
-- client reconnects to a different process. The row also IS the replay buffer,
-- replacing the per-process deque, so a reconnect resumes identically wherever
-- it lands. Swept by cleanup.py at DELTA_RETENTION_S -- this is a tail, not
-- history; `observation` is the history.
CREATE TABLE IF NOT EXISTS bay_delta (
  id         bigserial PRIMARY KEY,
  emitted_at timestamptz NOT NULL DEFAULT now(),
  payload    jsonb NOT NULL   -- exactly what goes on the wire, plus its cursor
);
CREATE INDEX IF NOT EXISTS bay_delta_emitted_idx ON bay_delta (emitted_at);
