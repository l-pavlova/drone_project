"""Parking as a 1-D resource along a curb: arclength, intervals, gaps.

`tools/make_runs.py` builds the runs; this module is the geometry and the counting
that sits on top of them. Nothing here knows about detectors or frames.

**Why a run and not a bay.** The published bay rectangles are wrong by more than a
bay width and no rigid correction fixes it (global shift 3.99 -> 3.59 m, per-street
-> 3.35 m, per-row-side -> 2.84 m, measured on the 56 hand-corrected bays of flight
0035). But the error is anisotropic -- cross-street 3.14 m against along-street
1.64 m -- and the two axes behave completely differently once parking is a line:

  * the CROSS-street component decides which run a car belongs to, and it is
    largely common-mode within a run, so one robust scalar per run absorbs it
    (`estimate_lateral`);
  * the ALONG-street component slides a car along the curb, and a gap LENGTH is
    invariant to sliding every car on a run by the same amount. A per-bay boolean
    is not: 3 m of along-street error flips it.

So this representation does not fix the geometry. It puts the irreducible error on
the axis where it costs nothing.

**A gap is not free parking until something has LOOKED at it.** Every count here
takes an `observed` mask and reports `observed_fraction` beside the answer, because
the one failure worse than saying nothing is telling a driver that unobserved curb
is empty curb. Note the mask must be radiometric, not merely geometric: a frame's
footprint says the curb was in shot, which is a different claim from the curb having
been visible -- flight 0074's `frame_0050.jpg` is a street essentially entirely
under summer canopy.
"""
import json
import math
import os

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
RUNS = os.path.join(DATA, "curb_runs.geojson")

CELL_M = 0.5         # evidence raster resolution along the curb
JOIN_BELOW_M = 1.5   # occupied stretches closer than this are one car row, not a gap
CLEARANCE_M = 0.5    # manoeuvring room a driver needs on top of the car length
CAR_LEN_M = 4.4      # median unprojected car length on this footage

# One definition, two consumers: `run_summary` turns a counted car instance back
# into the stretch of kerb it occupies, and `vision/score_runs.py` divides voted
# occupied length by it for the comparison prediction. Duplicated constants are
# this project's standing hazard (ORIGIN, DS_*, the four copies of the camera
# intrinsics), so it lives here and is imported.


def _enu():
    """score_occupancy's ENU, imported lazily so this module stays cheap."""
    import score_occupancy as so
    return so


def load(path=None, verified_only=True):
    """-> [run], ENU metres. `verified_only` drops chains of <4 bays, which have
    too few points to establish an axis and are marked UNVERIFIED by make_runs."""
    so = _enu()
    doc = json.load(open(path or RUNS, encoding="utf-8"))
    out = []
    for f in doc["features"]:
        pr = f["properties"]
        if verified_only and not pr.get("verified"):
            continue
        line = [so.to_enu(lon, lat) for lon, lat in f["geometry"]["coordinates"]]
        r = dict(pr)
        r["line"] = line
        r["s"] = _arclength(line)
        r["length_m"] = r["s"][-1]
        out.append(r)
    return out


def _arclength(line):
    s = [0.0]
    for a, b in zip(line, line[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return s


def locate(run, x, y):
    """(s, lateral, dist) of a ground point against a run.

    `s` is arclength from the run's start; `lateral` is signed, positive to the
    left of the direction of travel; `dist` is the true distance to the polyline.
    Returns the closest point over all segments, so a curving run is handled
    without any special case.

    **Gate on `dist`, never on `lateral`.** `lateral` is measured against the
    segment's INFINITE line, so a car sitting 20 m past the end of a run but
    collinear with it reads `lateral` ~ 0 and clamps to `s = length`. That is not
    a near miss, it is a different street -- and gating on `lateral` silently
    assigned 50 such detections to one 45 m run, every one of them a degenerate
    zero-length interval at the endpoint, which then reported the run as entirely
    unoccupied while claiming 42 cars had been seen on it.
    """
    best = None
    line, s = run["line"], run["s"]
    for i in range(len(line) - 1):
        ax, ay = line[i]
        bx, by = line[i + 1]
        dx, dy = bx - ax, by - ay
        dd = dx * dx + dy * dy
        if dd <= 0:
            continue
        t = ((x - ax) * dx + (y - ay) * dy) / dd
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        if best is None or d < best[0]:
            n = math.sqrt(dd)
            lat = ((x - ax) * (-dy) + (y - ay) * dx) / n
            best = (d, s[i] + t * n, lat)
    if best is None:
        return None
    return best[1], best[2], best[0]


def estimate_lateral(run, pts, max_m=6.0, min_n=5):
    """One robust scalar: how far this run sits from where the cars actually are.

    THE registration step, and the only one attempted. `pts` are ground points
    (detected car centres). Returns (delta, n, mad) or (0.0, n, None) when too few
    points support a fit -- never a guess, because a wrong shift is worse than none.

    The along-run component is deliberately NOT estimated. It was measured at rms
    2.36 m with a biased median of -1.37 m, so it is real; but a car's position
    within its own bay is genuinely uniform, which makes cars the wrong instrument
    for it, and the interval representation is what makes it not matter.
    """
    lats = []
    for x, y in pts:
        got = locate(run, x, y)
        if got is None:
            continue
        s, lat, dist = got
        if dist <= max_m:
            lats.append(lat)
    if len(lats) < min_n:
        return 0.0, len(lats), None
    lats.sort()
    med = lats[len(lats) // 2]
    dev = sorted(abs(v - med) for v in lats)
    return med, len(lats), dev[len(dev) // 2]


def shift_lateral(run, delta):
    """Return a copy of `run` moved `delta` metres to its left.

    A copy, never an edit: corrections in this project are sidecars that every
    consumer opts into, so an uncorrected result stays reproducible beside a
    corrected one. Same posture as `vision/bay_corrections.py`.
    """
    line, out = run["line"], []
    for i, (x, y) in enumerate(line):
        j = min(i, len(line) - 2)
        ax, ay = line[j]
        bx, by = line[j + 1]
        d = math.hypot(bx - ax, by - ay) or 1.0
        out.append((x + -(by - ay) / d * delta, y + (bx - ax) / d * delta))
    r = dict(run)
    r["line"] = out
    r["s"] = _arclength(out)
    r["length_m"] = r["s"][-1]
    r["lateral_shift_m"] = round(delta, 3)
    return r


def merge_intervals(ivs, join_below=JOIN_BELOW_M):
    """Sort, clip to >=0 and union, joining stretches separated by less than
    `join_below` -- that is inter-car spacing, not somewhere to park."""
    ivs = sorted((min(a, b), max(a, b)) for a, b in ivs)
    out = []
    for a, b in ivs:
        if out and a - out[-1][1] < join_below:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [(a, b) for a, b in out]


MIN_VIEWS = 3        # looks at a cell below which it is UNOBSERVED, not free


def vote_cells(run, frames, cell=CELL_M, min_views=MIN_VIEWS):
    """Many frames' intervals -> ONE pair of (occupied, observed) interval lists.

    -> (occupied, observed, views) where `views` is the per-cell look count.

    **A union across frames is the wrong accumulator and inflates occupancy.**
    The same parked car unprojects to a slightly different arclength in every view
    (measured scatter ~1 m), so unioning ten views of one car produces an interval
    a couple of metres longer than the car -- which eats the gaps on both sides of
    it and reports parking as fuller than it is. The bias is one-directional and it
    grows with the number of views, i.e. it is worst exactly where the evidence is
    best.

    So each 0.5 m cell of curb votes: it is occupied when a STRICT MAJORITY of the
    frames that looked at it saw a car there. That is deliberately the same rule as
    `detect_occupancy.vote()` and `recompute_states` in the web tier -- if the run
    layer and the bay layer resolved multi-view disagreement differently, their two
    numbers would stop being comparable, which is the whole point of reporting both.
    """
    length = run["length_m"]
    n = max(1, int(math.ceil(length / cell)))
    views = [0] * n
    hits = [0] * n

    def cells(a, b):
        return range(max(0, int(a / cell)), min(n, int(math.ceil(b / cell))))

    for fr in frames:
        touched = set()
        for a, b in fr.get("observed", ()):
            touched.update(cells(a, b))
        for k in touched:
            views[k] += 1
        seen = set()
        for a, b in fr.get("occupied", ()):
            seen.update(cells(a, b))
        # a car outside every observed span is a contradiction, not evidence
        for k in seen & touched:
            hits[k] += 1

    # A cell seen once or twice is thin evidence in BOTH directions, and the
    # asymmetry matters: calling it "observed and empty" advertises a parking
    # space on two looks, while calling it unobserved merely declines to answer.
    # Measured on flight 0035, cells carry a median of 3 views on a briefly-seen
    # run against 9 on a well-covered one, so this threshold is load-bearing.
    # It is a KNOB, and it must be tuned against `vision/data/gt_dji_0035.json`
    # once that exists -- never against the run layer's own output, which is the
    # circularity this whole redesign exists to escape.
    occupied = _runs_of(
        lambda k: views[k] >= min_views and hits[k] * 2 > views[k], n, cell, length)
    observed = _runs_of(lambda k: views[k] >= min_views, n, cell, length)
    return occupied, observed, views


def _runs_of(pred, n, cell, length):
    """Contiguous cells satisfying `pred` -> [(s0, s1), ...].

    Clamped to `length`: the cell grid is ceil(length / cell) cells, so the last
    one overruns the end of the run and an unclamped span reported an
    `observed_fraction` of 1.01 -- a number that cannot mean anything and that
    would quietly propagate into any coverage figure derived from it.
    """
    out, start = [], None
    for k in range(n):
        if pred(k):
            if start is None:
                start = k
        elif start is not None:
            out.append((start * cell, min(k * cell, length)))
            start = None
    if start is not None:
        out.append((start * cell, min(n * cell, length)))
    return out


def free_gaps(run, occupied, observed=None, space_m=None,
              clearance_m=CLEARANCE_M):
    """-> {free, gaps, occupied_len, observed_len, observed_fraction, capacity}.

    `occupied` and `observed` are lists of (s0, s1) in run arclength. A gap counts
    only where it is BOTH unoccupied and observed. `space_m` defaults to the run's
    own measured pitch, which is what makes this work for perpendicular parking
    (median pitch 2.35 m) as well as parallel (5.98 m) with no special case.

    A gap that runs off the end of the observed stretch is not charged the
    manoeuvring clearance, because its true length is unknown and may be larger.
    """
    length = run["length_m"]
    space = space_m or run.get("pitch_m") or 5.5
    obs = merge_intervals(observed, 0.0) if observed else [(0.0, length)]
    occ = merge_intervals(occupied)

    gaps, free = [], 0
    for o0, o1 in obs:
        cursor = o0
        for a, b in occ:
            if b <= o0 or a >= o1:
                continue
            a, b = max(a, o0), min(b, o1)
            if a - cursor > 0:
                gaps.append((cursor, a))
            cursor = max(cursor, b)
        if o1 - cursor > 0:
            gaps.append((cursor, o1))

    kept = []
    for a, b in gaps:
        g = b - a
        # only an interior gap is bounded by cars on both sides, so only it needs
        # the clearance charged against it
        interior = a > 1e-6 and b < length - 1e-6
        n = int((g - clearance_m) // space) if interior else int(g // space)
        if n > 0:
            kept.append({"s0": round(a, 2), "s1": round(b, 2),
                         "len_m": round(g, 2), "spaces": n})
            free += n

    occ_len = sum(b - a for a, b in occ)
    obs_len = sum(b - a for a, b in obs)

    # Sofiaplan's capacity is authoritative for how many spaces EXIST -- it is the
    # one thing in that dataset the drone cannot see and that its coordinates being
    # wrong does not spoil. A gap-derived count above it means the pitch estimate
    # is off, not that a space appeared, so the count is clamped.
    cap = run.get("capacity")
    capped = cap is not None and free > cap
    if capped:
        free = cap

    return {
        "run_id": run.get("run_id"),
        "capacity": cap,
        "free": free,
        "capped_to_capacity": capped,
        "gaps": kept,
        "occupied_len_m": round(occ_len, 2),
        "observed_len_m": round(obs_len, 2),
        "observed_fraction": round(obs_len / length, 3) if length else 0.0,
        "length_m": round(length, 2),
        "space_m": round(space, 2),
    }


def cluster_points(pts, cluster_m=2.0, min_views=2):
    """[(x, y, view_key), ...] -> [(x, y, n_views), ...], one entry per CAR.

    Single-link clustering. A parked car is seen from many frames and unprojects
    to nearly the same ground point each time (measured scatter ~1 m on flight
    0035), so merging within `cluster_m` recovers instances; `min_views` counts
    DISTINCT view keys, which is what separates a car from a one-frame false
    positive. Distinct and not raw count: ten boxes from one frame are one look,
    not ten.

    The single implementation, shared by the offline scorer
    (`detect_occupancy.cluster_instances`) and the web recompute, so a demo and a
    reported number can never diverge on how a car was counted.
    """
    n = len(pts)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    grid = {}
    for i, (x, y, _k) in enumerate(pts):
        grid.setdefault((int(x // cluster_m), int(y // cluster_m)), []).append(i)
    for i, (x, y, _k) in enumerate(pts):
        gx, gy = int(x // cluster_m), int(y // cluster_m)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((gx + dx, gy + dy), ()):
                    if j <= i:
                        continue
                    if math.hypot(x - pts[j][0], y - pts[j][1]) <= cluster_m:
                        ra, rb = find(i), find(j)
                        if ra != rb:
                            parent[rb] = ra

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    out = []
    for members in groups.values():
        if len({pts[i][2] for i in members}) < min_views:
            continue
        out.append((sum(pts[i][0] for i in members) / len(members),
                    sum(pts[i][1] for i in members) / len(members),
                    len(members)))
    return out


def _on_observed(s, obs, length):
    """Is arclength `s` inside a span anything looked at?

    The spans are half-open so that adjacent ones do not double-count a cell, but
    the END of the run is a real position a car can occupy and the last span
    closes exactly on it. Testing `a <= s < b` therefore dropped every car at
    `s == length` -- and `locate` CLAMPS anything past the end to exactly that
    value, so those are precisely the cars sitting at the tail of a kerb. Measured
    on flight 0035 it silently lost 3 of 49 placed instances (6%), which is the
    class of loss the whole run layer exists to stop doing.

    A car still outside every span is a genuine contradiction -- a detection on
    kerb the geometry says was never in frame -- and stays dropped, but the caller
    is told how many via `cars_off_observed` rather than left to infer it.
    """
    s = min(s, length - 1e-9)
    return any(a <= s < b for a, b in obs)


def car_intervals(car_s, car_m=CAR_LEN_M, join_below=JOIN_BELOW_M):
    """Counted car instances -> the stretches of kerb they occupy.

    Each car's arclength becomes an interval `car_m` long centred on it, then the
    set is merged. This is the ONLY route from occupancy to intervals on the
    published path, and it deliberately does not go through `vote_cells`.

    **Why not the cell vote.** The voted-occupied-length route loses cars twice --
    once when a cell fails the strict majority, again when two adjacent cars merge
    into one interval that divides to fewer than two -- and measured on flight
    0035's 18 hand-counted segments it recovers 19 of 29 cars with certainty of
    under-counting (bias -0.53, 95% CI [-0.91, -0.15]), against 34 and an unbiased
    +0.28 for instance counting. So the count comes from the instances and the
    geometry is reconstructed from the count, not the other way round.

    The reconstruction is approximate by construction -- a car's true extent along
    the kerb is not measured, only its centre -- which is exactly why it is used
    for GAPS and never for the car count. A gap length is what a driver needs and
    it survives every car being a few decimetres off; a count derived from length
    does not.
    """
    return merge_intervals([(s - car_m / 2.0, s + car_m / 2.0) for s in car_s],
                           join_below)


def run_summary(run, car_s, observed, space_m=None, car_m=CAR_LEN_M):
    """The product answer for one run: how many cars, and WHERE the free kerb is.

    `car_s` are arclengths of DISTINCT cars on this run (from
    `detect_occupancy.cluster_instances`, not raw detections); `observed` are the
    (s0, s1) spans anything actually looked at.

    **Cars are counted as INSTANCES, not derived from occupied length.** Dividing
    the voted occupied length by a car length was the first version and it is
    measurably worse -- see `car_intervals` for the numbers.

    **`free` is a MEASUREMENT, not a subtraction.** It used to be
    `capacity_observed - cars`, which can advertise a space that does not
    physically exist: four badly-spaced cars on a 30 m run leave the subtraction
    reporting 2 free while the actual gaps are 1.5 m each and nothing fits. A
    driver feels that error directly. So the cars are turned back into intervals
    and `free_gaps` counts what fits between them, which also yields `gaps` --
    where on the kerb to go, which the subtraction cannot express at all.
    The old number is kept beside it as `free_by_subtraction` rather than being
    replaced in silence, so the two stay comparable in the record.

    Capacity is scaled to the OBSERVED stretch. A flight that saw a third of a run
    cannot speak for the other two thirds, and reporting the run's full capacity as
    though it had would invent free spaces out of curb nobody looked at.

    Note the observed mask here is the raw merged spans, with no `MIN_VIEWS` gate
    (`vote_cells` applies one to its own observed output). Tightening it would move
    `observed_fraction` and `capacity_observed` at the same time as `free`, so it
    is deliberately left as a separate, single-variable question.
    """
    length = run["length_m"]
    space = space_m or run.get("pitch_m") or 5.5
    obs = merge_intervals(observed, 0.0) if observed else [(0.0, length)]
    obs_len = sum(b - a for a, b in obs)

    on_obs = [s for s in car_s if _on_observed(s, obs, length)]
    cars = len(on_obs)
    cap_obs = int(round(obs_len / space)) if space else 0
    # Sofiaplan's count is authoritative for the whole run, so the observed slice
    # can never imply more spaces than the run has.
    cap_obs = min(cap_obs, run.get("capacity") or cap_obs)

    occupied = car_intervals(on_obs, car_m)
    measured = free_gaps(run, occupied, obs, space_m=space)
    # `free_gaps` clamps to the run's FULL capacity; the published answer may only
    # speak for the stretch that was observed, so it is clamped again. A run whose
    # observed slice is too short to hold one space reports 0 free, not a space
    # measured off kerb nobody looked at.
    free = min(measured["free"], cap_obs)

    return {
        "run_id": run.get("run_id"),
        "street": run.get("street"),
        "capacity": run.get("capacity"),
        "capacity_observed": cap_obs,
        "cars": cars,
        "cars_off_observed": len(car_s) - cars,
        "free": free,
        "free_by_subtraction": max(0, cap_obs - cars),
        "gaps": measured["gaps"],
        "occupied_len_m": measured["occupied_len_m"],
        "observed_len_m": round(obs_len, 2),
        "observed_fraction": round(obs_len / length, 3) if length else 0.0,
        "length_m": round(length, 2),
        "space_m": round(space, 2),
    }


def bay_slots(run):
    """The derived per-bay view: bay k occupies [k*pitch, (k+1)*pitch].

    Keeps the existing map, the per-bay scorer and the whole web tier working off
    the run layer, and keeps bay ids addressable -- but they are RENDERED from the
    run, not the other way round. Order matches `bay_ids`, which make_runs wrote
    in arclength order.
    """
    p = run.get("pitch_m")
    ids = run.get("bay_ids") or []
    if not p or not ids:
        return []
    # the polyline was extended half a pitch past the end centroids, so bay k's
    # slot starts at k*pitch from the extended start
    return [(bid, i * p, (i + 1) * p) for i, bid in enumerate(ids)]


def overlaps(a0, a1, b0, b1):
    return min(a1, b1) - max(a0, b0)
