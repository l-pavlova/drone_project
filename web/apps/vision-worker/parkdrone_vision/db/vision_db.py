"""Postgres access for the vision worker (psycopg2).

Loads bay geometry as ENU rings, appends observations, and recomputes bay_state
as a majority vote over a bay's accumulated observations.
"""
import json

import psycopg2
import psycopg2.extras

from ..config import DATABASE_URL
from ..vision.scoring import to_enu


def connect():
    return psycopg2.connect(DATABASE_URL)


def load_bays_enu(conn):
    """All bays as {"id": str, "ring": [(x, y), ...]} in ENU metres.

    Ring is projected from WGS84 with the shared to_enu, dropping the closing
    vertex to match `load_bays` in score_occupancy.py.
    """
    bays = []
    with conn.cursor() as cur:
        cur.execute("SELECT bay_id, ST_AsGeoJSON(geom) FROM bay")
        for bay_id, geom_json in cur.fetchall():
            coords = json.loads(geom_json)["coordinates"][0]
            ring = [to_enu(lon, lat) for lon, lat in coords[:-1]]
            bays.append({"id": bay_id, "ring": ring})
    return bays


def insert_observations(conn, world, frame_idx, scores, gt=None):
    """Append one observation row per scored bay (single view)."""
    if not scores:
        return
    def fl(v):
        # coerce numpy scalars (np.float64) to native float for psycopg2
        return None if v is None else float(v)

    rows = []
    for s in scores:
        f = s["feat"]
        rows.append(
            (
                s["bay_id"],
                s["occupied"],
                frame_idx,
                world,
                1 if s["occupied"] else 0,  # votes_occupied (this view)
                1,                          # views (this view)
                fl(f.get("vis")),
                fl(s["off"]),
                fl(f.get("core_paint_frac")),
                fl(f.get("core_dark_frac")),
                fl(f.get("core_chroma")),
                fl(f.get("core_brightness")),
                fl(f.get("core_std")),
                None if gt is None else gt.get(s["bay_id"]),
            )
        )
    with conn.cursor() as cur:
        psycopg2.extras.execute_values(
            cur,
            """INSERT INTO observation
                 (bay_id, occupied, frame_idx, world, votes_occupied, views, vis,
                  center_off_px, core_paint_frac, core_dark_frac, core_chroma,
                  core_brightness, core_std, gt)
               VALUES %s""",
            rows,
        )


def recompute_states(conn, world, bay_ids, frame_idx):
    """Recompute bay_state (majority vote) for the given bays; upsert and return
    the deltas for bays whose occupancy flipped or became known for the first time.

    delta = {"bay_id", "occupied", "confidence", "updated_at"}
    """
    deltas = []
    with conn.cursor() as cur:
        for bay_id in bay_ids:
            cur.execute(
                """SELECT count(*) AS n,
                          coalesce(sum(CASE WHEN occupied THEN 1 ELSE 0 END), 0) AS occ
                     FROM observation WHERE bay_id = %s AND world = %s""",
                (bay_id, world),
            )
            n, occ = cur.fetchone()
            if n == 0:
                continue
            new_occ = occ * 2 > n           # strict majority sees a car
            confidence = max(occ, n - occ) / n

            cur.execute("SELECT occupied FROM bay_state WHERE bay_id = %s", (bay_id,))
            prior = cur.fetchone()
            changed = prior is None or prior[0] != new_occ

            cur.execute(
                """INSERT INTO bay_state
                     (bay_id, occupied, confidence, last_frame, source, updated_at)
                   VALUES (%s, %s, %s, %s, 'vision', now())
                   ON CONFLICT (bay_id) DO UPDATE SET
                     occupied = EXCLUDED.occupied,
                     confidence = EXCLUDED.confidence,
                     last_frame = EXCLUDED.last_frame,
                     source = 'vision',
                     updated_at = now()
                   RETURNING updated_at""",
                (bay_id, new_occ, confidence, frame_idx),
            )
            updated_at = cur.fetchone()[0]
            if changed:
                deltas.append(
                    {
                        "bay_id": bay_id,
                        "occupied": new_occ,
                        "confidence": round(confidence, 3),
                        "updated_at": updated_at.isoformat(),
                    }
                )
    return deltas
