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
                      'last_frame', s.last_frame, 'updated_at', s.updated_at, 'source', s.source
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
                      COUNT(*) FILTER (WHERE {_FRESH} AND s.occupied IS FALSE) AS free,
                      COUNT(*) FILTER (WHERE {_FRESH} AND s.occupied IS TRUE)  AS occupied,
                      COUNT(*) FILTER (WHERE NOT ({_FRESH}) OR s.occupied IS NULL) AS unknown
                 FROM bay b LEFT JOIN bay_state s USING (bay_id)
                GROUP BY b.zona
                ORDER BY b.zona""",
            {"window_s": OCCUPANCY_WINDOW_S},
        )
        rows = cur.fetchall()
    return [
        {"zona": z, "free": int(free), "occupied": int(occ), "unknown": int(unk)}
        for (z, free, occ, unk) in rows
    ]


def detail(conn, bay_id, history_limit=20):
    """One bay + current state + recent observations (default 20), or None."""
    with conn.cursor() as cur:
        cur.execute(
            f"""SELECT b.bay_id, b.zona, b.street, b.park_txt, b.bearing_deg, b.public,
                      ST_AsGeoJSON(b.geom)::json AS geometry,
                      CASE WHEN {_FRESH} THEN s.occupied END,
                      CASE WHEN {_FRESH} THEN s.confidence END,
                      s.last_frame, s.updated_at, s.source
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
    """Rows per frame_job status, plus the age of the oldest still-queued job.

    A growing `oldest_queued_age_s` is the stall signal: work is arriving that
    the classify threads are not finishing.
    """
    with conn.cursor() as cur:
        cur.execute("SELECT status, COUNT(*) FROM frame_job GROUP BY status")
        counts = {status: int(n) for status, n in cur.fetchall()}
        cur.execute(
            """SELECT EXTRACT(EPOCH FROM now() - MIN(enqueued_at))
                 FROM frame_job WHERE status = 'queued'"""
        )
        oldest = cur.fetchone()[0]
    return {
        "queued": counts.get("queued", 0),
        "processed": counts.get("processed", 0),
        "failed": counts.get("failed", 0),
        "oldest_queued_age_s": round(float(oldest), 1) if oldest is not None else None,
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
                       COUNT(*) FILTER (WHERE {_FRESH} AND s.occupied IS TRUE),
                       COUNT(*) FILTER (WHERE {_FRESH} AND s.occupied IS FALSE)
                  FROM bay b LEFT JOIN bay_state s USING (bay_id)""",
            {"window_s": OCCUPANCY_WINDOW_S},
        )
        total, occupied, free = cur.fetchone()
    total, occupied, free = int(total), int(occupied), int(free)
    return {
        "bays_total": total,
        "bays_occupied": occupied,
        "bays_free": free,
        "bays_unknown": total - occupied - free,
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
    """Idempotent frame insert on (drone_id, survey_area, frame_idx).

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
               ON CONFLICT (drone_id, survey_area, frame_idx) DO NOTHING
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
            "SELECT frame_id FROM frame WHERE drone_id = %s AND survey_area = %s AND frame_idx = %s",
            (drone_id, survey_area, pose["frame_idx"]),
        )
        return cur.fetchone()[0], True


def mark_frame_processed(conn, frame_id):
    """Mark a frame's job scored so startup crash-recovery won't re-enqueue it."""
    with conn.cursor() as cur:
        cur.execute(
            "UPDATE frame_job SET status = 'processed', finished_at = now() WHERE frame_id = %s",
            (frame_id,),
        )
    conn.commit()


def mark_frame_failed(conn, frame_id):
    """Mark a frame's job unrecoverable (e.g. its image expired from the store)
    so crash-recovery stops re-enqueuing it every restart."""
    with conn.cursor() as cur:
        cur.execute(
            "UPDATE frame_job SET status = 'failed', finished_at = now() WHERE frame_id = %s",
            (frame_id,),
        )
    conn.commit()


def unscored_frames(conn):
    """Frames whose job is still queued (never scored) — the recovery backlog.

    Reconstructs the classify-job payload by joining the work state in
    frame_job back to the immutable pose/payload facts in frame, so an
    in-memory queue lost on restart is rebuilt from Postgres + S3.
    """
    with conn.cursor() as cur:
        cur.execute(
            """SELECT f.frame_id, f.survey_area, f.frame_idx, f.x, f.y, f.alt, f.yaw,
                      f.roll, f.pitch, f.cam_pitch, f.cam_roll, f.image_uri
                 FROM frame_job j JOIN frame f USING (frame_id)
                WHERE j.status = 'queued' ORDER BY j.enqueued_at"""
        )
        rows = cur.fetchall()
    jobs = []
    for (frame_id, survey_area, frame_idx, x, y, alt, yaw, roll, pitch,
         cam_pitch, cam_roll, image_uri) in rows:
        # The gimbal angles are part of the pose for projection purposes, so a
        # recovered job must carry them or it would classify this frame
        # differently from the live path. Absent (pre-0008 rows, or a drone that
        # reports none) means "assume nadir", which is what project() falls back
        # to -- so they are only set when actually known.
        pose = {"frame_idx": frame_idx, "x": x, "y": y, "alt": alt,
                "yaw": yaw, "roll": roll, "pitch": pitch}
        if cam_pitch is not None:
            pose["cam_pitch"] = cam_pitch
        if cam_roll is not None:
            pose["cam_roll"] = cam_roll
        jobs.append(
            {
                "frame_id": frame_id,
                "survey_area": survey_area,
                "frame_idx": frame_idx,
                "pose": pose,
                "image_uri": image_uri,
            }
        )
    return jobs
