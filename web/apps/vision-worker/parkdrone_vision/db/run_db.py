"""Curb-run occupancy: write per-frame evidence, recompute run_state, read it back.

The run layer's counterpart to `vision_db.py`, and deliberately a separate module:
`bay`/`observation`/`bay_state` are untouched by any of this, the per-bay path
still runs on every frame, and the two answers are served side by side. The sim,
the golden fixtures and every accuracy number on record are per-bay; folding the
run layer into that code would make the two incomparable at exactly the moment we
want to compare them.

**Why it exists.** Hand-counted ground truth for flight 0035 (18 segments, ~280 m,
29 cars) puts `bay_votes_from_dets` at 9 cars -- 31% -- with certain under-counting
(bias -1.11, 95% CI [-1.50, -0.78]). The run layer recovers significantly more
(+0.58 cars/segment, CI [+0.21, +0.96]). See `vision/score_runs.py`.

**Cars are counted as instances, and the vote is a CLUSTER, not a majority.** A
parked car is seen in many frames and unprojects to nearly the same ground point
each time, so clustering those points recovers one car per car. Deriving a count
from occupied length instead loses cars twice over and under-counts with certainty.
`curb_runs.cluster_points` owns that clustering, shared with the offline scorer, so
the demo and the reported number cannot diverge.
"""
import json

from .. import config
from ..vision import runs as run_geom


def _rows(cur):
    cols = [d[0] for d in cur.description]
    return [dict(zip(cols, r)) for r in cur.fetchall()]


def record_frame_runs(conn, world, frame_idx, frame_id, mission_id, dets, spans):
    """Replace this frame's run evidence. Returns the run_ids it touched.

    `dets` is [(run_id, s, x, y, conf), ...]; `spans` is [(run_id, s0, s1), ...].

    DELETE-then-INSERT keyed on `frame_id` is what makes it idempotent, and that
    matters on one replica as much as on several: `process_frame` commits the
    evidence and marks the job finished in separate transactions, so a crash
    between them re-scores the frame. Appending twice would double one car into
    two clusters only if the noise were large -- but it would certainly double the
    observed spans, and with them the observed capacity a free count is derived
    from. A frame with no `frame_id` (the offline replay) simply appends.
    """
    touched = set()
    with conn.cursor() as cur:
        if frame_id is not None:
            cur.execute("DELETE FROM run_detection WHERE frame_id = %s", (frame_id,))
            cur.execute("DELETE FROM run_observation WHERE frame_id = %s", (frame_id,))
        for run_id, s0, s1 in spans:
            touched.add(run_id)
            cur.execute(
                """INSERT INTO run_observation
                     (run_id, world, mission_id, frame_idx, frame_id, s0, s1)
                   VALUES (%s, %s, %s, %s, %s, %s, %s)""",
                (run_id, world, mission_id, frame_idx, frame_id, s0, s1))
        for run_id, s, x, y, conf in dets:
            touched.add(run_id)
            cur.execute(
                """INSERT INTO run_detection
                     (run_id, world, mission_id, frame_idx, frame_id, s, x, y, conf)
                   VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)""",
                (run_id, world, mission_id, frame_idx, frame_id, s, x, y, conf))
    return sorted(touched)


# The freshness window and the newest-mission rule are the bay layer's, copied
# deliberately rather than shared: if the two layers windowed differently, the
# same flight would produce two answers that could not be compared, which is the
# whole point of serving both. See vision_db._RECOMPUTE_SQL.
_WINDOW = "observed_at > now() - (%s || ' seconds')::interval"

_NEWEST_MISSION = f"""
  SELECT mission_id FROM run_observation
   WHERE run_id = %s AND {_WINDOW}
   ORDER BY observed_at DESC LIMIT 1
"""


def recompute_run_states(conn, world, run_ids, backend=None):
    """Re-derive `run_state` for the given runs. -> [{run_id, cars, free, ...}]

    Only the NEWEST mission's evidence counts, exactly as `bay_state` resolves it:
    two flights can fall inside one freshness window, and a kerb that emptied
    between them must not keep reporting the earlier flight's cars.
    """
    window = config.OCCUPANCY_WINDOW_S
    out = []
    with conn.cursor() as cur:
        for run_id in run_ids:
            geom = run_geom.get(run_id)
            if geom is None:
                continue
            cur.execute(_NEWEST_MISSION, (run_id, window))
            row = cur.fetchone()
            if row is None:
                continue
            mission = row[0]
            mission_sql = ("mission_id IS NOT DISTINCT FROM %s")

            cur.execute(
                f"""SELECT s0, s1 FROM run_observation
                     WHERE run_id = %s AND {_WINDOW} AND {mission_sql}""",
                (run_id, window, mission))
            spans = [(r[0], r[1]) for r in cur.fetchall()]

            cur.execute(
                f"""SELECT x, y, frame_idx FROM run_detection
                     WHERE run_id = %s AND {_WINDOW} AND {mission_sql}""",
                (run_id, window, mission))
            pts = [(r[0], r[1], r[2]) for r in cur.fetchall()]

            summary = run_geom.summarize(geom, pts, spans)
            cur.execute(
                """INSERT INTO run_state
                     (run_id, world, mission_id, cars, free, capacity_observed,
                      observed_fraction, backend, gaps, free_by_subtraction,
                      updated_at)
                   VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s, now())
                   ON CONFLICT (run_id) DO UPDATE SET
                     world = EXCLUDED.world, mission_id = EXCLUDED.mission_id,
                     cars = EXCLUDED.cars, free = EXCLUDED.free,
                     capacity_observed = EXCLUDED.capacity_observed,
                     observed_fraction = EXCLUDED.observed_fraction,
                     backend = EXCLUDED.backend, gaps = EXCLUDED.gaps,
                     free_by_subtraction = EXCLUDED.free_by_subtraction,
                     updated_at = now()""",
                (run_id, world, mission, summary["cars"], summary["free"],
                 summary["capacity_observed"], summary["observed_fraction"],
                 backend, json.dumps(summary["gaps"]),
                 summary["free_by_subtraction"]))
            out.append(dict(summary, run_id=run_id))
    return out


_FRESH = ("rs.updated_at > now() - (%s || ' seconds')::interval")


def feature_collection(conn, bbox=None):
    """Runs as GeoJSON LineStrings, carrying their current state.

    `cars`/`free` are gated on freshness the same way `occupied` is: a state older
    than the window is reported as unknown (nulls) rather than as a stale answer,
    derived at read time so nothing has to sweep.
    """
    window = config.OCCUPANCY_WINDOW_S
    where, extra = ["cr.verified"], []
    if bbox:
        where.append("cr.geom && ST_MakeEnvelope(%s,%s,%s,%s,4326)")
        extra += [bbox["minLon"], bbox["minLat"], bbox["maxLon"], bbox["maxLat"]]
    sql = f"""
      SELECT cr.run_id, cr.street, cr.capacity, cr.pitch_m, cr.length_m,
             cr.park_txt, cr.zona,
             CASE WHEN {_FRESH} THEN rs.cars END              AS cars,
             CASE WHEN {_FRESH} THEN rs.free END              AS free,
             CASE WHEN {_FRESH} THEN rs.capacity_observed END AS capacity_observed,
             CASE WHEN {_FRESH} THEN rs.observed_fraction END AS observed_fraction,
             CASE WHEN {_FRESH} THEN rs.backend END           AS backend,
             CASE WHEN {_FRESH} THEN rs.gaps END               AS gaps,
             rs.updated_at,
             ST_AsGeoJSON(cr.geom) AS gj
        FROM curb_run cr
        LEFT JOIN run_state rs ON rs.run_id = cr.run_id
       WHERE {' AND '.join(where)}
    """
    # _FRESH is repeated once per gated column, and each copy takes the window
    params = [window] * 6 + extra
    with conn.cursor() as cur:
        cur.execute(sql, params)
        rows = _rows(cur)
    feats = []
    for r in rows:
        gj = r.pop("gj")
        ts = r.pop("updated_at")
        feats.append({
            "type": "Feature",
            "properties": dict(r, updated_at=ts.isoformat() if ts else None),
            "geometry": json.loads(gj),
        })
    return {"type": "FeatureCollection", "features": feats}


def summary(conn, world=None):
    """Flight-level totals for the ops view and the demo caption."""
    window = config.OCCUPANCY_WINDOW_S
    sql = f"""
      SELECT count(*)                                        AS runs,
             coalesce(sum(CASE WHEN {_FRESH} THEN rs.cars END), 0)  AS cars,
             coalesce(sum(CASE WHEN {_FRESH} THEN rs.free END), 0)  AS free,
             coalesce(sum(CASE WHEN {_FRESH} THEN rs.capacity_observed END), 0)
                                                             AS capacity_observed,
             coalesce(sum(cr.capacity), 0)                    AS capacity_total
        FROM run_state rs JOIN curb_run cr ON cr.run_id = rs.run_id
       WHERE (%s IS NULL OR rs.world = %s)
    """
    with conn.cursor() as cur:
        cur.execute(sql, [window] * 3 + [world, world])
        return _rows(cur)[0]
