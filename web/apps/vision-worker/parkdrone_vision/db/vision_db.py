"""Postgres access for the vision worker (psycopg2).

Loads bay geometry as ENU rings, appends observations, and recomputes bay_state
as a majority vote over the observations inside OCCUPANCY_WINDOW_S.
"""
import json
import math

import numpy as np
import psycopg2
import psycopg2.extras

from ..config import OCCUPANCY_WINDOW_S, required
from ..vision.scoring import to_enu


def connect():
    return psycopg2.connect(required("DATABASE_URL"))


def load_bays_enu(conn):
    """All bays as {"id": str, "ring": [(x, y), ...]} in ENU metres.

    Ring is projected from WGS84 with the shared to_enu, dropping the closing
    vertex to match `load_bays` in score_occupancy.py.

    Each bay also carries `cx`/`cy` (ring centroid) and `r` (circumradius — the
    farthest vertex from that centroid), which `score_frame` uses to reject bays
    nowhere near the camera footprint without projecting them. `load_bays` in
    score_occupancy.py stores cx/cy the same way.

    Returns (bays, index) where `index` is a BayIndex holding the same centroids
    and radii as numpy arrays, so the rejection is one vectorised comparison
    rather than a Python loop over every bay in the city.
    """
    bays = []
    with conn.cursor() as cur:
        cur.execute("SELECT bay_id, ST_AsGeoJSON(geom) FROM bay")
        for bay_id, geom_json in cur.fetchall():
            coords = json.loads(geom_json)["coordinates"][0]
            ring = [to_enu(lon, lat) for lon, lat in coords[:-1]]
            cx = sum(p[0] for p in ring) / len(ring)
            cy = sum(p[1] for p in ring) / len(ring)
            r = max(math.hypot(p[0] - cx, p[1] - cy) for p in ring)
            bays.append({"id": bay_id, "ring": ring, "cx": cx, "cy": cy, "r": r})
    return bays, BayIndex(bays)


class BayIndex:
    """Centroids + circumradii of a bay set, as numpy arrays.

    Built once at startup; `visible()` narrows a frame's candidates from "every
    bay in the dataset" to "the handful under the camera". Kept next to
    load_bays_enu because it is derived from exactly the same geometry.
    """

    def __init__(self, bays):
        self.cx = np.array([b["cx"] for b in bays], dtype=np.float64)
        self.cy = np.array([b["cy"] for b in bays], dtype=np.float64)
        self.r = np.array([b["r"] for b in bays], dtype=np.float64)

    def visible(self, bays, x, y, reach):
        """Bays whose bounding circle can reach within `reach` metres of (x, y).

        Conservative by construction: a bay every vertex of which is farther than
        `reach + its own radius` cannot put any vertex inside the frame, so this
        never drops a bay that would have scored.
        """
        d2 = (self.cx - x) ** 2 + (self.cy - y) ** 2
        lim = reach + self.r
        return [bays[i] for i in np.nonzero(d2 <= lim * lim)[0]]


def insert_observations(conn, survey_area, frame_idx, scores, gt=None,
                        mission_id=None, backend=None):
    """Append one observation row per scored bay (single view).

    `mission_id` is what lets the vote tell two flights apart (migration 0009).
    It stays optional: a caller with no mission records NULL, and the vote groups
    all mission-less observations of a bay together exactly as it did before.

    `backend` (migration 0010) records WHICH model produced the verdict. The
    colour statistics below are the heuristic's; on the detector backend `feat`
    carries only `det_score` and the rest come out NULL, which is why the
    backend has to be stated rather than guessed from which columns are set.
    """
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
                survey_area,
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
                mission_id,
                backend,
                fl(f.get("det_score")),
            )
        )
    with conn.cursor() as cur:
        psycopg2.extras.execute_values(
            cur,
            """INSERT INTO observation
                 (bay_id, occupied, frame_idx, survey_area, votes_occupied, views, vis,
                  center_off_px, core_paint_frac, core_dark_frac, core_chroma,
                  core_brightness, core_std, gt, mission_id, backend, det_score)
               VALUES %s""",
            rows,
        )


# Recompute every touched bay in ONE statement instead of 3 queries per bay in a
# Python loop. Two things make that possible:
#
#   * every CTE sees the same snapshot, so `prior` reads bay_state as it was
#     BEFORE `ups` writes it — that is what lets change detection happen in the
#     same pass as the upsert;
#   * the whole thing is atomic, so two writers can no longer interleave a
#     read-then-write on the same bay.
#
# The vote only counts observations inside OCCUPANCY_WINDOW_S, so cost is bounded
# by the window rather than growing with the bay's accumulated history.
_RECOMPUTE_SQL = """
WITH latest AS (
  -- Which FLIGHT saw each bay most recently, inside the window. A survey area
  -- can be re-flown at any time, and the window is hours wide, so without this
  -- a fresh look would be averaged against a stale one from the previous
  -- flight -- a bay that emptied between flights would keep voting "occupied"
  -- on the strength of history. The newest mission wins the bay outright;
  -- earlier observations stay as history, which is what they are for.
  SELECT DISTINCT ON (bay_id) bay_id, mission_id
    FROM observation
   WHERE bay_id = ANY(%(bay_ids)s)
     AND survey_area = %(survey_area)s
     AND observed_at > now() - make_interval(secs => %(window_s)s)
   ORDER BY bay_id, observed_at DESC
), agg AS (
  SELECT o.bay_id,
         count(*)                          AS n,
         count(*) FILTER (WHERE o.occupied) AS occ,
         -- Provenance of the flight that now owns this bay. `latest` already
         -- pinned it to one mission, and a mission runs on one backend, so the
         -- max() is picking the single value present rather than resolving a
         -- disagreement.
         max(o.backend)                     AS backend
    FROM observation o
    JOIN latest l ON l.bay_id = o.bay_id
                 -- IS NOT DISTINCT FROM, not =, so the pre-0009 rows (mission
                 -- NULL) group together instead of matching nothing at all.
                 AND o.mission_id IS NOT DISTINCT FROM l.mission_id
   WHERE o.bay_id = ANY(%(bay_ids)s)
     AND o.survey_area = %(survey_area)s
     AND o.observed_at > now() - make_interval(secs => %(window_s)s)
   GROUP BY o.bay_id
), calc AS (
  SELECT bay_id,
         (occ * 2 > n)                        AS occupied,   -- strict majority sees a car
         GREATEST(occ, n - occ)::real / n     AS confidence,
         backend
    FROM agg
), prior AS (
  SELECT bay_id, occupied FROM bay_state WHERE bay_id = ANY(%(bay_ids)s)
), ups AS (
  INSERT INTO bay_state (bay_id, occupied, confidence, last_frame, source, backend, updated_at)
  SELECT bay_id, occupied, confidence, %(frame_idx)s, 'vision', backend, now() FROM calc
  ON CONFLICT (bay_id) DO UPDATE SET
    occupied   = EXCLUDED.occupied,
    confidence = EXCLUDED.confidence,
    last_frame = EXCLUDED.last_frame,
    source     = 'vision',
    backend    = EXCLUDED.backend,
    updated_at = now()
  RETURNING bay_id, occupied, confidence, updated_at
)
SELECT u.bay_id, u.occupied, u.confidence, u.updated_at
  FROM ups u LEFT JOIN prior p USING (bay_id)
 WHERE p.bay_id IS NULL OR p.occupied IS DISTINCT FROM u.occupied
"""


def recompute_states(conn, survey_area, bay_ids, frame_idx, window_s=None):
    """Recompute bay_state (majority vote over the freshness window) for the given
    bays; upsert and return the deltas for bays whose occupancy flipped or became
    known for the first time.

    A bay with no observations inside the window contributes no row to `agg`, so
    it is simply not upserted — its existing state is left alone and ages out on
    the read side instead.

    delta = {"bay_id", "occupied", "confidence", "updated_at"}
    """
    if not bay_ids:
        return []
    with conn.cursor() as cur:
        cur.execute(
            _RECOMPUTE_SQL,
            {
                "bay_ids": list(bay_ids),
                "survey_area": survey_area,
                "frame_idx": frame_idx,
                "window_s": OCCUPANCY_WINDOW_S if window_s is None else window_s,
            },
        )
        return [
            {
                "bay_id": bay_id,
                "occupied": occupied,
                "confidence": round(confidence, 3),
                "updated_at": updated_at.isoformat(),
            }
            for bay_id, occupied, confidence, updated_at in cur.fetchall()
        ]
