-- Split `frame` into two tables, each with one job.
--
-- `frame` was doing two things at once: recording an immutable fact (this drone
-- uploaded these pixels from this pose at this time) AND tracking mutable work
-- state (queued -> processed/failed). Those have opposite lifecycles: the fact
-- is written once and never changes, the work state is updated on every frame
-- and is only interesting until it reaches a terminal value.
--
-- After this migration:
--   frame      = the ingest ledger. Append-only, never UPDATEd.
--   frame_job  = the durable classify queue. It is what lets the in-process
--                job queue survive a restart: startup re-enqueues every row
--                still 'queued', rebuilt from frame + the object store.
--
-- The 1:1 FK keeps the pairing exact — a job cannot exist without its frame,
-- and ON DELETE CASCADE preserves the "clear the frame rows to reprocess a
-- survey area" workflow, which now takes the jobs with it.

CREATE TABLE frame_job (
  frame_id     text PRIMARY KEY REFERENCES frame(frame_id) ON DELETE CASCADE,
  status       text NOT NULL DEFAULT 'queued'
                 CHECK (status IN ('queued', 'processed', 'failed')),
  enqueued_at  timestamptz NOT NULL DEFAULT now(),
  finished_at  timestamptz
);

-- Carry existing lifecycle across before the columns go. `received_at` is the
-- best available enqueue time: pre-split, the row was committed as 'queued' in
-- the same statement that set received_at.
INSERT INTO frame_job (frame_id, status, enqueued_at, finished_at)
SELECT frame_id, status, received_at, processed_at FROM frame;

-- Mirrors the old frame_status_idx: this is the crash-recovery scan
-- (WHERE status = 'queued' ORDER BY enqueued_at), now over a narrow table.
CREATE INDEX frame_job_status_idx ON frame_job(status, enqueued_at);

-- Dropping the columns also drops frame_status_idx, which indexed frame(status,
-- received_at) — no separate DROP INDEX needed.
ALTER TABLE frame DROP COLUMN status;
ALTER TABLE frame DROP COLUMN processed_at;
