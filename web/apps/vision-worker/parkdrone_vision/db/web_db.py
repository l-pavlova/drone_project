"""Web-edge SQL: read queries + ingest/mission writes + drone auth lookups.

All functions take a psycopg2 connection (borrowed from pool.py). Geospatial
predicates push down into PostGIS; the app layer never materialises the full
bay set.

psycopg2 usually returns json/jsonb columns as already-parsed Python objects,
but _asjson guards the case where a build returns text (double-encoding).
"""
import json

from ..config import OCCUPANCY_WINDOW_S

# A bay_state row older than the freshness window is reported as unknown rather
# than as its last known value: "this spot was free two hours ago" is not an
# answer a driver can act on, and the schema already spells unknown as a NULL
# `occupied`. Derived at read time on purpose — a sweeper that NULLed rows would
# rewrite the whole table on every tick and lose the last reading for good.
_FRESH = "s.updated_at > now() - make_interval(secs => %(window_s)s)"

# A street closure is an EXTERNAL authoritative fact that overrides what the
# camera saw (TODO #13, migration 0012): roadworks, a market, an accident. An
# empty bay on a closed street is not available parking, however clearly the
# detector saw that it was empty.
#
# Derived at read time and expressed as one shared fragment for the same reason
# _FRESH is: the four read paths below must not be able to disagree about what
# "closed" means, and nothing is written into bay_state - the observations stay
# exactly as the camera recorded them, and only the published answer moves. That
# is what lets a closure be lifted without re-flying the street.
#
# NULL bound = open: no valid_from means already in force, no valid_to means no
# known end date.
_CLOSED = """EXISTS (SELECT 1 FROM street_closure c
                      WHERE ST_Intersects(c.geom, b.centroid)
                        AND (c.valid_from IS NULL OR c.valid_from <= now())
                        AND (c.valid_to   IS NULL OR c.valid_to   >  now()))"""

# The reason to show in the popup, when there is more than one overlapping
# closure the earliest-starting one wins so the answer is stable.
_CLOSURE_INFO = """(SELECT json_build_object('closure_id', c.closure_id,
                                             'label', c.label,
                                             'reason', c.reason,
                                             'valid_to', c.valid_to)
                      FROM street_closure c
                     WHERE ST_Intersects(c.geom, b.centroid)
                       AND (c.valid_from IS NULL OR c.valid_from <= now())
                       AND (c.valid_to   IS NULL OR c.valid_to   >  now())
                     ORDER BY c.valid_from NULLS FIRST, c.closure_id
                     LIMIT 1)"""


def _asjson(v):
    """Coerce a PostGIS json column to a Python object (parse if it came as text)."""
    return json.loads(v) if isinstance(v, (str, bytes, bytearray)) else v


# ---- reads -----------------------------------------------------------------

def feature_collection(conn, bbox=None, zona=None):
    """Bays (+ current state) as a GeoJSON FeatureCollection. Unknown = no state row."""
    env = (
        "ST_MakeEnvelope(%(minLon)s, %(minLat)s, %(maxLon)s, %(maxLat)s, 4326)"
        if bbox
        else "NULL"
    )
    sql = f"""SELECT json_build_object(
                'type', 'FeatureCollection',
                'features', COALESCE(json_agg(feat), '[]'::json)
              ) AS fc
         FROM (
           SELECT json_build_object(
                    'type', 'Feature',
                    'geometry', ST_AsGeoJSON(b.geom)::json,
                    'properties', json_build_object(
                      'bay_id', b.bay_id, 'zona', b.zona, 'street', b.street,
                      'park_txt', b.park_txt, 'bearing_deg', b.bearing_deg, 'public', b.public,
                      'occupied', CASE WHEN {_FRESH} THEN s.occupied END,
                      'confidence', CASE WHEN {_FRESH} THEN s.confidence END,
                      'last_frame', s.last_frame, 'updated_at', s.updated_at, 'source', s.source,
                      -- Which model decided this bay (migration 0010). Gated on
                      -- freshness like `occupied` is: naming a backend beside a
                      -- verdict that has already expired would attribute a claim
                      -- nothing is currently making.
                      'backend', CASE WHEN {_FRESH} THEN s.backend END,
                      -- Street closure (0012). NOT gated on freshness: a
                      -- closure is asserted by an authority, not observed by
                      -- the drone, so it does not go stale when the flight does
                      -- - it ends when its own validity window ends.
                      'closed', {_CLOSED},
                      'closure', {_CLOSURE_INFO}
                    )
                  ) AS feat
             FROM bay b
             LEFT JOIN bay_state s USING (bay_id)
            WHERE ({env} IS NULL OR b.geom && {env})
              AND (%(zona)s::text IS NULL OR b.zona = %(zona)s)
         ) t"""
    params = {"zona": zona, "window_s": OCCUPANCY_WINDOW_S}
    if bbox:
        params.update(bbox)
    with conn.cursor() as cur:
        cur.execute(sql, params)
        row = cur.fetchone()
    return _asjson(row[0]) if row and row[0] else {"type": "FeatureCollection", "features": []}


def summary(conn):
    """Free/occupied/unknown counts grouped by zone (stale rows count as unknown)."""
    with conn.cursor() as cur:
        cur.execute(
            f"""SELECT b.zona,
                      COUNT(*) FILTER (WHERE NOT {_CLOSED} AND {_FRESH}
                                             AND s.occupied IS FALSE) AS free,
                      COUNT(*) FILTER (WHERE NOT {_CLOSED} AND {_FRESH}
                                             AND s.occupied IS TRUE)  AS occupied,
                      COUNT(*) FILTER (WHERE NOT {_CLOSED}
                                             AND (NOT ({_FRESH}) OR s.occupied IS NULL))
                                                                      AS unknown,
                      -- A closed bay is its own count, taken out of the other
                      -- three: folding it into `unknown` would say the survey
                      -- failed to see it, and into `free` would advertise
                      -- parking that does not exist.
                      COUNT(*) FILTER (WHERE {_CLOSED})               AS closed
                 FROM bay b LEFT JOIN bay_state s USING (bay_id)
                GROUP BY b.zona
                ORDER BY b.zona""",
            {"window_s": OCCUPANCY_WINDOW_S},
        )
        rows = cur.fetchall()
    return [
        {"zona": z, "free": int(free), "occupied": int(occ), "unknown": int(unk),
         "closed": int(cl)}
        for (z, free, occ, unk, cl) in rows
    ]


def closures(conn, bbox=None, active_only=True):
    """Street closures as a GeoJSON FeatureCollection (migration 0012).

    Unlike the no-fly zones this is NOT static reference data parsed from a file
    - a closure is created and lifted while the system runs, and the bay read
    paths join against it - so it lives in Postgres and is queried per request.
    """
    env = (
        "ST_MakeEnvelope(%(minLon)s, %(minLat)s, %(maxLon)s, %(maxLat)s, 4326)"
        if bbox
        else "NULL"
    )
    active = """AND (c.valid_from IS NULL OR c.valid_from <= now())
                AND (c.valid_to   IS NULL OR c.valid_to   >  now())""" if active_only else ""
    sql = f"""SELECT json_build_object(
                'type', 'FeatureCollection',
                'features', COALESCE(json_agg(feat), '[]'::json)
              ) AS fc
         FROM (
           SELECT json_build_object(
                    'type', 'Feature',
                    'geometry', ST_AsGeoJSON(c.geom)::json,
                    'properties', json_build_object(
                      'closure_id', c.closure_id, 'label', c.label,
                      'reason', c.reason, 'source', c.source,
                      'valid_from', c.valid_from, 'valid_to', c.valid_to,
                      -- how many bays this closure actually takes off the map:
                      -- the same ST_Intersects the bay reads use, so the number
                      -- in the popup cannot disagree with the map
                      'bays', (SELECT COUNT(*) FROM bay b
                                WHERE ST_Intersects(c.geom, b.centroid))
                    )
                  ) AS feat
             FROM street_closure c
            WHERE ({env} IS NULL OR c.geom && {env})
              {active}
         ) t"""
    params = dict(bbox or {})
    with conn.cursor() as cur:
        cur.execute(sql, params)
        row = cur.fetchone()
    return _asjson(row[0]) if row and row[0] else {"type": "FeatureCollection", "features": []}


def bays_in_closure(conn, closure_id):
    """Bay ids a closure covers — the set whose published answer just changed."""
    with conn.cursor() as cur:
        cur.execute(
            """SELECT b.bay_id FROM bay b JOIN street_closure c
                      ON ST_Intersects(c.geom, b.centroid)
                WHERE c.closure_id = %s""",
            (closure_id,),
        )
        return [r[0] for r in cur.fetchall()]


def detail(conn, bay_id, history_limit=20):
    """One bay + current state + recent observations (default 20), or None."""
    with conn.cursor() as cur:
        cur.execute(
            f"""SELECT b.bay_id, b.zona, b.street, b.park_txt, b.bearing_deg, b.public,
                      ST_AsGeoJSON(b.geom)::json AS geometry,
                      CASE WHEN {_FRESH} THEN s.occupied END,
                      CASE WHEN {_FRESH} THEN s.confidence END,
                      s.last_frame, s.updated_at, s.source,
                      CASE WHEN {_FRESH} THEN s.backend END,
                      {_CLOSED}, {_CLOSURE_INFO}
                 FROM bay b LEFT JOIN bay_state s USING (bay_id)
                WHERE b.bay_id = %(bay_id)s""",
            {"bay_id": bay_id, "window_s": OCCUPANCY_WINDOW_S},
        )
        bay = cur.fetchone()
        if bay is None:
            return None
        cols = [
            "bay_id", "zona", "street", "park_txt", "bearing_deg", "public",
            "geometry", "occupied", "confidence", "last_frame", "updated_at", "source",
            "backend", "closed", "closure",
        ]
        out = dict(zip(cols, bay))
        out["geometry"] = _asjson(out["geometry"])
        cur.execute(
            """SELECT occupied, frame_idx, survey_area, votes_occupied, views, vis, gt, observed_at
                 FROM observation WHERE bay_id = %s ORDER BY observed_at DESC LIMIT %s""",
            (bay_id, history_limit),
        )
        hcols = ["occupied", "frame_idx", "survey_area", "votes_occupied", "views", "vis", "gt", "observed_at"]
        out["history"] = [dict(zip(hcols, r)) for r in cur.fetchall()]
    return out


def nearest_bay(conn, lon, lat):
    """bay_id nearest a lon/lat (GiST <-> on centroid), or None."""
    with conn.cursor() as cur:
        cur.execute(
            """SELECT bay_id FROM bay
                ORDER BY centroid <-> ST_SetSRID(ST_MakePoint(%s, %s), 4326)
                LIMIT 1""",
            (lon, lat),
        )
        row = cur.fetchone()
    return row[0] if row else None


def bay_exists(conn, bay_id):
    with conn.cursor() as cur:
        cur.execute("SELECT 1 FROM bay WHERE bay_id = %s", (bay_id,))
        return cur.fetchone() is not None


def upsert_state(conn, bay_id, occupied, confidence, last_frame, source):
    """Upsert current occupancy (manual dev override; vision writes via db.recompute_states)."""
    with conn.cursor() as cur:
        cur.execute(
            """INSERT INTO bay_state (bay_id, occupied, confidence, last_frame, source, updated_at)
               VALUES (%s, %s, %s, %s, %s, now())
               ON CONFLICT (bay_id) DO UPDATE SET
                 occupied = EXCLUDED.occupied,
                 confidence = EXCLUDED.confidence,
                 last_frame = EXCLUDED.last_frame,
                 source = EXCLUDED.source,
                 updated_at = now()
               RETURNING updated_at""",
            (bay_id, occupied, confidence, last_frame, source),
        )
        return cur.fetchone()[0]


# ---- operational metrics (GET /api/v1/metrics) -----------------------------
#
# The durable half of the metrics payload: what the DB knows and the process
# does not. Job outcomes and ingest times are already recorded by the pipeline
# (`frame_job.status`/`finished_at`, `frame.received_at`) — these queries only
# surface them. All of them are bounded scans: `frame_job` and `frame` are swept
# to FRAME_RETENTION_S by cleanup.py, and the time filters ride
# frame_received_idx / frame_job_status_idx.

def job_counts(conn):
    """Rows per frame_job status, plus the two stall signals.

    `oldest_queued_age_s` keeps the meaning it always had — work that NO replica
    has picked up — which is why 'running' is a separate count rather than being
    folded into 'queued': a claimed job is being worked on, and counting it as
    backlog would make a healthy pipeline look stalled.

    `expired_leases` is the new signal that only exists once work is claimed: a
    job whose holder stopped renewing, i.e. a replica that died mid-frame. It
    self-heals (the next claim reclaims it), so a persistently non-zero value is
    the alert, not a single blip.
    """
    with conn.cursor() as cur:
        cur.execute("SELECT status, COUNT(*) FROM frame_job GROUP BY status")
        counts = {status: int(n) for status, n in cur.fetchall()}
        cur.execute(
            """SELECT EXTRACT(EPOCH FROM now() - MIN(enqueued_at))
                 FROM frame_job WHERE status = 'queued'"""
        )
        oldest = cur.fetchone()[0]
        cur.execute(
            """SELECT COUNT(*) FROM frame_job
                WHERE status = 'running' AND lease_expires_at < now()"""
        )
        expired = int(cur.fetchone()[0])
    return {
        "queued": counts.get("queued", 0),
        "running": counts.get("running", 0),
        "processed": counts.get("processed", 0),
        "failed": counts.get("failed", 0),
        "oldest_queued_age_s": round(float(oldest), 1) if oldest is not None else None,
        "expired_leases": expired,
    }


def ingest_rates(conn, window_s):
    """Frames ingested in the last minute / hour / window, and the newest one."""
    with conn.cursor() as cur:
        cur.execute(
            """SELECT COUNT(*) FILTER (WHERE received_at > now() - interval '1 minute'),
                      COUNT(*) FILTER (WHERE received_at > now() - interval '1 hour'),
                      COUNT(*) FILTER (WHERE received_at > now() - make_interval(secs => %(w)s)),
                      MAX(received_at)
                 FROM frame""",
            {"w": window_s},
        )
        last_min, last_hour, in_window, newest = cur.fetchone()
    return {
        "frames_last_1m": int(last_min),
        "frames_last_1h": int(last_hour),
        "frames_in_window": int(in_window),
        "frames_per_min_window": round(int(in_window) * 60.0 / window_s, 2),
        "last_frame_at": newest,
    }


def classify_latency(conn, window_s):
    """End-to-end job latency (enqueue → finish) over the window, in seconds.

    This is queue wait + classify time, i.e. what a stalled or oversubscribed
    pipeline actually shows up in — not the per-frame CV cost, which the
    in-process counters report separately.
    """
    with conn.cursor() as cur:
        cur.execute(
            """SELECT COUNT(*),
                      AVG(EXTRACT(EPOCH FROM finished_at - enqueued_at)),
                      PERCENTILE_CONT(0.5) WITHIN GROUP (
                        ORDER BY EXTRACT(EPOCH FROM finished_at - enqueued_at)),
                      PERCENTILE_CONT(0.95) WITHIN GROUP (
                        ORDER BY EXTRACT(EPOCH FROM finished_at - enqueued_at)),
                      COUNT(*) FILTER (WHERE status = 'failed')
                 FROM frame_job
                WHERE finished_at > now() - make_interval(secs => %(w)s)""",
            {"w": window_s},
        )
        n, avg, p50, p95, failed = cur.fetchone()
    n = int(n)
    r = lambda v: round(float(v), 3) if v is not None else None  # noqa: E731
    return {
        "finished_in_window": n,
        "avg_s": r(avg),
        "p50_s": r(p50),
        "p95_s": r(p95),
        "failed_in_window": int(failed),
        "failure_rate": round(int(failed) / n, 4) if n else None,
    }


def fleet_health(conn, window_s):
    """Drone registry + mission progress — the "is data flowing" half."""
    with conn.cursor() as cur:
        cur.execute(
            """SELECT COUNT(*),
                      COUNT(*) FILTER (WHERE last_seen > now() - make_interval(secs => %(w)s))
                 FROM drone""",
            {"w": window_s},
        )
        drones_total, drones_active = cur.fetchone()
        cur.execute(
            """SELECT m.mission_id, m.drone_id, m.survey_area, m.started_at,
                      m.frames_done, m.frames_expected,
                      EXTRACT(EPOCH FROM now() - MAX(f.received_at)) AS since_last_frame_s
                 FROM mission m LEFT JOIN frame f USING (mission_id)
                WHERE m.ended_at IS NULL
                GROUP BY m.mission_id
                ORDER BY m.started_at DESC
                LIMIT 20"""
        )
        cols = ["mission_id", "drone_id", "survey_area", "started_at",
                "frames_done", "frames_expected", "since_last_frame_s"]
        missions = []
        for row in cur.fetchall():
            m = dict(zip(cols, row))
            idle = m["since_last_frame_s"]
            m["since_last_frame_s"] = round(float(idle), 1) if idle is not None else None
            missions.append(m)
    return {
        "drones_total": int(drones_total),
        "drones_active": int(drones_active),
        "missions_active": len(missions),
        "missions": missions,
    }


def model_accuracy(conn):
    """Classifier accuracy where ground truth exists (sim/eval runs).

    Two different questions, both worth showing, because they can disagree:

    * **per view** — of all labelled (bay, frame) classifications, how many were
      right. This is the raw classifier, single look, no voting.
    * **per bay** — of the bays that carry a label, how many ended up in the
      right *state* after the windowed majority vote. This is the number the
      product is actually judged on, and it is normally higher: voting is what
      cancels a bad view.

    Both return None in production rather than 0 — there is no ground truth
    there, and an accuracy of "unknown" must not render as a failing score.
    """
    with conn.cursor() as cur:
        cur.execute(
            """SELECT COUNT(*), COUNT(*) FILTER (WHERE occupied = gt)
                 FROM observation WHERE gt IS NOT NULL"""
        )
        views, views_ok = cur.fetchone()
        # One label per bay (its most recent), compared against the voted state.
        cur.execute(
            """WITH labelled AS (
                 SELECT DISTINCT ON (bay_id) bay_id, gt
                   FROM observation
                  WHERE gt IS NOT NULL
                  ORDER BY bay_id, observed_at DESC
               )
               SELECT COUNT(*), COUNT(*) FILTER (WHERE s.occupied = l.gt)
                 FROM labelled l JOIN bay_state s USING (bay_id)"""
        )
        bays, bays_ok = cur.fetchone()
    views, views_ok, bays, bays_ok = int(views), int(views_ok), int(bays), int(bays_ok)
    return {
        "views_scored": views,
        "views_correct": views_ok,
        "view_accuracy": round(views_ok / views, 4) if views else None,
        "bays_scored": bays,
        "bays_correct": bays_ok,
        "state_accuracy": round(bays_ok / bays, 4) if bays else None,
    }


def coverage_counts(conn):
    """Bay totals split by freshness — the same rule every read path applies."""
    with conn.cursor() as cur:
        cur.execute(
            f"""SELECT COUNT(*),
                       COUNT(*) FILTER (WHERE NOT {_CLOSED} AND {_FRESH}
                                              AND s.occupied IS TRUE),
                       COUNT(*) FILTER (WHERE NOT {_CLOSED} AND {_FRESH}
                                              AND s.occupied IS FALSE),
                       COUNT(*) FILTER (WHERE {_CLOSED})
                  FROM bay b LEFT JOIN bay_state s USING (bay_id)""",
            {"window_s": OCCUPANCY_WINDOW_S},
        )
        total, occupied, free, closed = cur.fetchone()
    total, occupied, free, closed = int(total), int(occupied), int(free), int(closed)
    return {
        "bays_total": total,
        "bays_occupied": occupied,
        "bays_free": free,
        "bays_closed": closed,
        # Still derived by subtraction, so it must have every other count taken
        # out of it - including the closed ones, or a closure would silently
        # read as "the survey never saw this bay".
        "bays_unknown": total - occupied - free - closed,
    }


# ---- auth ------------------------------------------------------------------

def lookup_drone(conn, api_key_hash):
    with conn.cursor() as cur:
        cur.execute("SELECT drone_id FROM drone WHERE api_key_hash = %s", (api_key_hash,))
        row = cur.fetchone()
    return row[0] if row else None


def bump_last_seen(conn, drone_id):
    with conn.cursor() as cur:
        cur.execute("UPDATE drone SET last_seen = now() WHERE drone_id = %s", (drone_id,))


# ---- ingest / mission ------------------------------------------------------

def insert_mission(conn, mission_id, drone_id, survey_area, area, frames_expected):
    with conn.cursor() as cur:
        cur.execute(
            """INSERT INTO mission (mission_id, drone_id, survey_area, area, frames_expected)
               VALUES (%s, %s, %s, %s, %s)""",
            (mission_id, drone_id, survey_area, area, frames_expected),
        )


def end_mission(conn, mission_id, drone_id):
    with conn.cursor() as cur:
        cur.execute(
            "UPDATE mission SET ended_at = now() WHERE mission_id = %s AND drone_id = %s",
            (mission_id, drone_id),
        )


def bump_mission_done(conn, mission_id):
    with conn.cursor() as cur:
        cur.execute(
            "UPDATE mission SET frames_done = frames_done + 1 WHERE mission_id = %s",
            (mission_id,),
        )


def insert_frame(conn, frame_id, drone_id, mission_id, survey_area, pose, image_uri):
    """Idempotent frame insert on (drone_id, mission, frame_idx) - migration 0009.

    Keyed on the MISSION, not the survey area, because `frame_idx` restarts at 0
    every flight: on the old key a re-flight of an area was silently ignored end
    to end (no job, no delta, a map that never moved, and no error). A re-send
    inside one flight is still a duplicate - that is the retry case the
    constraint exists for - while a new flight is new data.

    Frames with no mission fall back to the old (drone, area, frame_idx)
    behaviour via the index's COALESCE sentinel, so a client that never learned
    about missions keeps the semantics it was written against.

    `pose` is expected normalized (canonical `frame_idx` key) — the ingest
    handler does that at the edge, so this layer stays stdlib-only.

    A new frame also gets its frame_job row (status 'queued') in the SAME
    transaction, so a committed ledger entry always has a recoverable job — the
    crash window the durability design depends on stays closed. A duplicate
    creates no job, which is what makes a re-send not re-enqueue work.

    Returns (frame_id, duplicate): on conflict, the existing frame_id + True.
    """
    with conn.cursor() as cur:
        cur.execute(
            """INSERT INTO frame
                 (frame_id, drone_id, mission_id, survey_area, frame_idx, x, y, alt, yaw, roll, pitch,
                  cam_pitch, cam_roll, image_uri)
               VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
               ON CONFLICT (drone_id, COALESCE(mission_id, 'area:' || survey_area),
                            frame_idx) DO NOTHING
               RETURNING frame_id""",
            (
                frame_id, drone_id, mission_id, survey_area, pose["frame_idx"],
                pose["x"], pose["y"], pose["alt"], pose["yaw"],
                pose.get("roll"), pose.get("pitch"),
                # the gimbal angles matter to project(): without them a recovered
                # job would re-project this frame as if the camera were nadir
                pose.get("cam_pitch"), pose.get("cam_roll"), image_uri,
            ),
        )
        row = cur.fetchone()
        if row:
            cur.execute("INSERT INTO frame_job (frame_id) VALUES (%s)", (row[0],))
            return row[0], False
        cur.execute(
            """SELECT frame_id FROM frame
                WHERE drone_id = %s
                  AND COALESCE(mission_id, 'area:' || survey_area) = %s
                  AND frame_idx = %s""",
            (drone_id, mission_id or f"area:{survey_area}", pose["frame_idx"]),
        )
        return cur.fetchone()[0], True


def mark_frame_processed(conn, frame_id):
    """Mark a frame's job scored, and drop its lease so nothing reclaims it."""
    with conn.cursor() as cur:
        cur.execute(
            """UPDATE frame_job
                  SET status = 'processed', finished_at = now(),
                      lease_expires_at = NULL
                WHERE frame_id = %s""",
            (frame_id,),
        )
    conn.commit()


def mark_frame_failed(conn, frame_id, err=None):
    """Give up on a job permanently: its image is gone, or it has burned through
    MAX_ATTEMPTS. Terminal — the claim query never returns a 'failed' row again.

    `err` is kept because "why did this stop" is otherwise unanswerable after
    the fact: the exception only ever reached stdout, which is routinely lost.
    """
    with conn.cursor() as cur:
        cur.execute(
            """UPDATE frame_job
                  SET status = 'failed', finished_at = now(),
                      lease_expires_at = NULL, last_error = %s
                WHERE frame_id = %s""",
            (None if err is None else str(err)[:2000], frame_id),
        )
    conn.commit()


def release_frame_job(conn, frame_id, err=None):
    """Hand a job back after a TRANSIENT failure: queued again, lease dropped.

    The point is that some other replica (or this one) retries it in seconds
    rather than at the next restart. `attempts` is not touched here — it was
    already incremented by the claim, which is what makes MAX_ATTEMPTS count
    tries rather than failures.
    """
    with conn.cursor() as cur:
        cur.execute(
            """UPDATE frame_job
                  SET status = 'queued', claimed_by = NULL, claimed_at = NULL,
                      lease_expires_at = NULL, last_error = %s
                WHERE frame_id = %s""",
            (None if err is None else str(err)[:2000], frame_id),
        )
    conn.commit()


def _job_payload(row):
    """One frame_job+frame row -> the classify-job dict the workers consume."""
    (frame_id, survey_area, frame_idx, x, y, alt, yaw, roll, pitch,
     cam_pitch, cam_roll, image_uri, mission_id, reclaimed, attempts) = row
    # The gimbal angles are part of the pose for projection purposes, so a
    # claimed job must carry them or it would classify this frame differently
    # from the live path. Absent (pre-0008 rows, or a drone that reports none)
    # means "assume nadir", which is what project() falls back to -- so they are
    # only set when actually known.
    pose = {"frame_idx": frame_idx, "x": x, "y": y, "alt": alt,
            "yaw": yaw, "roll": roll, "pitch": pitch}
    if cam_pitch is not None:
        pose["cam_pitch"] = cam_pitch
    if cam_roll is not None:
        pose["cam_roll"] = cam_roll
    return {
        "frame_id": frame_id,
        "survey_area": survey_area,
        "frame_idx": frame_idx,
        # Carried for the same reason as the gimbal angles: the vote is scoped
        # per mission (0009), so a job must record the flight it came from or
        # its observations would join the wrong one -- and a reclaimed frame
        # must score identically to the same frame processed live.
        "mission_id": mission_id,
        "pose": pose,
        "image_uri": image_uri,
        # True when this claim took a lapsed lease off someone else, i.e. it is
        # recovery rather than fresh work. Counted separately in the metrics.
        "reclaimed": bool(reclaimed),
        # Tries INCLUDING this one (RETURNING reads the post-UPDATE value), so
        # the worker can tell a retryable failure from the last allowed one.
        "attempts": int(attempts),
    }


# One statement claims work and reads the payload back, because a claim that is
# not atomic with the read is not a claim. SKIP LOCKED is what makes N replicas
# (and N dispatcher polls) share the backlog instead of queueing behind each
# other on the same rows.
_CLAIM_SQL = """
WITH c AS (
    SELECT frame_id, (status = 'running') AS reclaimed
      FROM frame_job
     WHERE attempts < %(max_attempts)s
       AND (status = 'queued'
            -- the reaper, inline: a lease nobody renewed is abandoned work, and
            -- the replica that notices is by definition alive
            OR (status = 'running' AND lease_expires_at < now()))
     ORDER BY enqueued_at
     FOR UPDATE SKIP LOCKED
     LIMIT %(limit)s
)
UPDATE frame_job j
   SET status = 'running',
       claimed_by = %(owner)s,
       claimed_at = now(),
       lease_expires_at = now() + make_interval(secs => %(lease_s)s),
       attempts = j.attempts + 1
  FROM c JOIN frame f ON f.frame_id = c.frame_id
 WHERE j.frame_id = c.frame_id
RETURNING f.frame_id, f.survey_area, f.frame_idx, f.x, f.y, f.alt, f.yaw,
          f.roll, f.pitch, f.cam_pitch, f.cam_roll, f.image_uri,
          f.mission_id, c.reclaimed, j.attempts
"""


def claim_frames(conn, owner, limit, lease_s, max_attempts):
    """Claim up to `limit` unfinished frames for `owner`; returns job payloads.

    This replaces the old unscored_frames()/recover() pair, which selected every
    'queued' row with no ownership filter — safe only because exactly one
    process ever ran it. Claiming makes the same backlog shareable: two replicas
    booting together split it instead of both doing all of it, and a frame
    already being classified is invisible to everyone else until its lease runs
    out.
    """
    with conn.cursor() as cur:
        cur.execute(_CLAIM_SQL, {"owner": owner, "limit": limit,
                                 "lease_s": lease_s, "max_attempts": max_attempts})
        rows = cur.fetchall()
    conn.commit()
    return [_job_payload(r) for r in rows]


def claimable_count(conn, max_attempts):
    """Unfinished work any replica could pick up right now — the startup banner
    and the 'is the pipeline stalled' question, without claiming anything."""
    with conn.cursor() as cur:
        cur.execute(
            """SELECT count(*) FROM frame_job
                WHERE attempts < %(max_attempts)s
                  AND (status = 'queued'
                       OR (status = 'running' AND lease_expires_at < now()))""",
            {"max_attempts": max_attempts},
        )
        return cur.fetchone()[0]
