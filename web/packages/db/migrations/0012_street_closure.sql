-- Street closures: the first authoritative fact in this system that is NOT
-- derived from the camera (TODO #13).
--
-- A street can be closed - roadworks, a market, an accident, a parade. Until
-- now nothing in the chain knew the concept existed, so a closed street was
-- surveyed like any other and its bays were published as ordinary free
-- parking. An empty bay on a closed street is not available parking, so the
-- product was telling drivers something false.
--
-- Three decisions are encoded here, and each has an alternative that was
-- rejected for a reason worth keeping:
--
--   1. MATCHED BY GEOMETRY, NEVER BY STREET NAME. The obvious key is the name,
--      and it is the wrong one: bays carry only `mestopoloz` (a Cyrillic
--      display name, seeded into bay.street) and OSM roads carry `name`, and
--      the two are reconciled by a conservative substring rule that already
--      fails on 2 of the 49 streets in the 1 km cut (TODO #9). That rule lives
--      in Python; keying on it here would need a third copy of it, in SQL. A
--      polygon needs no reconciliation, is the same test in the generator and
--      in the server, and can cover half a street or a square - which a name
--      never can.
--
--   2. `bay_state` IS NOT TOUCHED. Putting a third state there would mean
--      migrating `occupied` off NOT NULL and widening the `source` CHECK, and
--      it would write a fact the camera never observed into the camera's own
--      record. The override is applied at READ time instead, exactly like the
--      freshness window in web_db._FRESH - the pattern this codebase already
--      chose, and for the same reason (a sweeper would rewrite the whole table
--      on every tick and lose the last real reading for good).
--
--   3. OBSERVATIONS ARE STILL RECORDED. Frames over a closed street are still
--      classified and still write observation rows. The closure overrides the
--      published ANSWER, not the record - which is what keeps it falsifiable,
--      and what lets the closure be lifted without re-flying.

CREATE TABLE street_closure (
  closure_id  text PRIMARY KEY,
  label       text,                       -- human street name, display only
  reason      text,                       -- shown in the map popup
  geom        geometry(Polygon, 4326) NOT NULL,
  -- NULL bound = open: valid_from NULL means "already in force", valid_to NULL
  -- means "no known end date". Deliberately a real interval rather than the
  -- single `permanent` boolean nofly.py collapses ED-269's applicability array
  -- to - that loses the schedule and leaves the UI unable to say WHEN a zone
  -- applies, which is a limitation worth not repeating.
  valid_from  timestamptz,
  valid_to    timestamptz,
  source      text,                       -- which authority says so
  created_at  timestamptz NOT NULL DEFAULT now()
);

-- Every read path intersects bay.centroid against active closures, so the
-- spatial index carries the join.
CREATE INDEX street_closure_geom_gix ON street_closure USING GIST (geom);
-- Partial index on the open-ended case: the common query is "active now", and
-- most closures in force have no end date.
CREATE INDEX street_closure_valid_idx ON street_closure(valid_from, valid_to);
