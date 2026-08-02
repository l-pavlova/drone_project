-- Index for the frame-retention sweep. No schema or data changes.
--
-- cleanup.py scans for frames past FRAME_RETENTION_S every CLEANUP_INTERVAL_S.
-- Without this it is a sequential scan of the whole ingest ledger on every pass,
-- and the ledger is the one table that grows with every frame ever flown.
CREATE INDEX IF NOT EXISTS frame_received_idx ON frame (received_at);
