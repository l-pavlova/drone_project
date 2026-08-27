"""Bridge to the offline curb-run geometry, and the per-frame extraction.

Same posture as `scoring.py`: the runs, the arclength maths and the clustering
live in `vision/curb_runs.py` + `vision/detect_occupancy.py` and are imported
verbatim, so the demo, the offline scorer and the reported accuracy number cannot
diverge on how a car is counted or where a run is.

Runs are reference data -- derived from the bays by `tools/make_runs.py`, changing
only when they are re-derived -- so they are parsed once and cached in memory here
rather than read from Postgres on the classify path. Same call as
`vision/ground_truth.py` and `nofly.py`. The `curb_run` TABLE exists so the read
API can do bbox queries in PostGIS; this cache is what the hot path uses.
"""
import os
import sys
import threading

_HERE = os.path.dirname(os.path.abspath(__file__))
_VISION_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "..", "..", "..", "..", "vision"))
if _VISION_DIR not in sys.path:
    sys.path.insert(0, _VISION_DIR)

import curb_runs as cr                                          # noqa: E402
import detect_occupancy as dio                                  # noqa: E402
import score_occupancy as so                                    # noqa: E402

RUNS_GEOJSON = os.environ.get("CURB_RUNS_GEOJSON")
LATERAL_MAX_M = float(os.environ.get("RUN_LATERAL_MAX_M", "6.0"))

_lock = threading.Lock()
_runs = None
_by_id = None


def _load():
    global _runs, _by_id
    with _lock:
        if _runs is None:
            try:
                _runs = cr.load(RUNS_GEOJSON)
            except FileNotFoundError:
                # No runs file is not an error: the run layer simply stays off and
                # the per-bay path is unaffected. Same tolerance nofly.py gives a
                # missing zones file -- an empty layer, never a 500.
                _runs = []
            _by_id = {r["run_id"]: r for r in _runs}
    return _runs


def all_runs():
    return _load()


def get(run_id):
    _load()
    return _by_id.get(run_id)


def frame_evidence(dets, pose, cam=None, lateral_max_m=LATERAL_MAX_M):
    """One frame's detections + pose -> (detections on runs, observed spans).

    -> ([(run_id, s, x, y, conf), ...], [(run_id, s0, s1), ...])

    The observed spans are what separate empty kerb from unobserved kerb. They are
    computed even when the frame has no detections at all -- that is precisely the
    frame that proves a stretch is free, and dropping it would make an empty street
    indistinguishable from one nobody flew over.

    Note the span test is GEOMETRIC: it says the kerb was inside the frame, not
    that it was visible. Under summer canopy those differ, so `observed_fraction`
    downstream is an upper bound.
    """
    runs = _load()
    if not runs:
        return [], []

    spans = []
    for r in runs:
        for s0, s1 in dio.observed_spans(r, pose, cam=cam):
            spans.append((r["run_id"], round(s0, 2), round(s1, 2)))

    out = []
    for box, conf in dets:
        g = so.unproject((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                         pose, cam=cam)
        if g is None:
            continue
        best = None
        for r in runs:
            got = cr.locate(r, g[0], g[1])
            if got is None:
                continue
            # Gate on distance to the POLYLINE, never on the signed lateral: a car
            # past the end of a run but collinear with it reads lateral ~0 and
            # clamps to s = length, which once assigned 50 detections to one 45 m
            # run as zero-length intervals at its endpoint.
            if got[2] <= lateral_max_m and (best is None or got[2] < best[0]):
                best = (got[2], r["run_id"], got[0])
        if best is not None:
            out.append((best[1], round(best[2], 2), g[0], g[1], conf))
    return out, spans


def summarize(run, pts, spans):
    """Clustered cars + observed spans -> the published per-run answer."""
    cars = cr.cluster_points(pts)
    car_s = []
    for x, y, _n in cars:
        got = cr.locate(run, x, y)
        if got is not None and got[2] <= LATERAL_MAX_M:
            car_s.append(got[0])
    return cr.run_summary(run, car_s, spans)
