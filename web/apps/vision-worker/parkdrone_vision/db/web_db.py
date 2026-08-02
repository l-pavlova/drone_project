"""Web-edge SQL: read queries + ingest/mission writes + drone auth lookups.

All functions take a psycopg2 connection (borrowed from pool.py). Geospatial
predicates push down into PostGIS; the app layer never materialises the full
bay set.

psycopg2 usually returns json/jsonb columns as already-parsed Python objects,
but _asjson guards the case where a build returns text (double-encoding).
"""
import json


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
                      'occupied', s.occupied, 'confidence', s.confidence,
                      'last_frame', s.last_frame, 'updated_at', s.updated_at, 'source', s.source
                    )
                  ) AS feat
             FROM bay b
             LEFT JOIN bay_state s USING (bay_id)
            WHERE ({env} IS NULL OR b.geom && {env})
              AND (%(zona)s::text IS NULL OR b.zona = %(zona)s)
         ) t"""
    params = {"zona": zona}
    if bbox:
        params.update(bbox)
    with conn.cursor() as cur:
        cur.execute(sql, params)
        row = cur.fetchone()
    return _asjson(row[0]) if row and row[0] else {"type": "FeatureCollection", "features": []}


def summary(conn):
    """Free/occupied/unknown counts grouped by zone."""
    with conn.cursor() as cur:
        cur.execute(
            """SELECT b.zona,
                      COUNT(*) FILTER (WHERE s.occupied IS FALSE) AS free,
                      COUNT(*) FILTER (WHERE s.occupied IS TRUE)  AS occupied,
                      COUNT(*) FILTER (WHERE s.occupied IS NULL)  AS unknown
                 FROM bay b LEFT JOIN bay_state s USING (bay_id)
                GROUP BY b.zona
                ORDER BY b.zona"""
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
            """SELECT b.bay_id, b.zona, b.street, b.park_txt, b.bearing_deg, b.public,
                      ST_AsGeoJSON(b.geom)::json AS geometry,
                      s.occupied, s.confidence, s.last_frame, s.updated_at, s.source
                 FROM bay b LEFT JOIN bay_state s USING (bay_id)
                WHERE b.bay_id = %s""",
            (bay_id,),
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
                 (frame_id, drone_id, mission_id, survey_area, frame_idx, x, y, alt, yaw, roll, pitch, image_uri)
               VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
               ON CONFLICT (drone_id, survey_area, frame_idx) DO NOTHING
               RETURNING frame_id""",
            (
                frame_id, drone_id, mission_id, survey_area, pose["frame_idx"],
                pose["x"], pose["y"], pose["alt"], pose["yaw"],
                pose.get("roll"), pose.get("pitch"), image_uri,
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
                      f.roll, f.pitch, f.image_uri
                 FROM frame_job j JOIN frame f USING (frame_id)
                WHERE j.status = 'queued' ORDER BY j.enqueued_at"""
        )
        rows = cur.fetchall()
    jobs = []
    for frame_id, survey_area, frame_idx, x, y, alt, yaw, roll, pitch, image_uri in rows:
        jobs.append(
            {
                "frame_id": frame_id,
                "survey_area": survey_area,
                "frame_idx": frame_idx,
                "pose": {"frame_idx": frame_idx, "x": x, "y": y, "alt": alt,
                         "yaw": yaw, "roll": roll, "pitch": pitch},
                "image_uri": image_uri,
            }
        )
    return jobs
