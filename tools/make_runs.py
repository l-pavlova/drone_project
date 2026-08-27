#!/usr/bin/env python3
"""Group parking bays into CURB RUNS -- the primitive that survives a bad map.

    python tools/make_runs.py                 # -> data/curb_runs.geojson
    python tools/make_runs.py --report        # print the runs, write nothing

Sofiaplan publishes parking spaces as POINTS, and `tools/make_bays.py` turns each
into a 5.4 x 2.2 m rectangle CENTRED on that point, taking only its orientation from
the nearest OSM centerline -- the lateral position is never constrained to anything.
Projected into real drone footage those rectangles land on sidewalks, on grass and in
the middle of the street, and only 122 of 386 hand-labelled cars (32%) fall within the
3 m assignment radius of a published centroid.

**No rigid correction fixes that.** Measured against the 56 hand-corrected bays of
flight 0035: a global shift takes the error 3.99 -> 3.59 m, a per-street shift ->
3.35 m, a per-row-side 2-D shift -> 2.84 m. All of them leave more than a bay width.
And the published rows are *perfectly* regular -- a straight-line fit within a run has
RMS 0.017 m -- only because `make_bays.py` drafted them that way; the real bays are
irregular and ~15% tighter in pitch (5.41 m published vs 4.48 m drawn over the same
ids), so a published row accumulates along-street drift that no offset can absorb.

What DOES survive is the shape of the error. It is anisotropic: cross-street 3.14 m
against along-street 1.64 m on an unbiased sample. A gap length measured along a curb
is invariant to a common-mode slide along that curb, while a per-bay boolean flips.
So the run -- a polyline with an arclength, a capacity and a pitch -- is the unit that
can carry an honest answer, and the individual rectangle is not.

**Runs are built from GEOMETRY, never from the street name.** The same rule the street
closures already follow: bays carry `mestopoloz`, roads carry OSM `name`, and
`norm_street()` substring reconciliation already fails on 2 of the 49 streets in the
1 km cut. A name is also the wrong key in principle here -- one street has two sides
that need opposite-signed corrections (on the two published rows of Джеймс Баучер,
12.1 m apart), so the row-side is the minimum honest unit and the name cannot express
it. The name rides along as an attribute, resolved by majority.

**Capacity is Sofiaplan's real contribution and is preserved exactly.** Every
individual coordinate in that dataset is distrusted here, but the COUNT of spaces on
a stretch of curb is the thing the city actually knows and the drone cannot see (a
bay is a bay whether or not a car is in it). It is carried as `capacity`, straight
from the number of points, and must NOT be recomputed as length/pitch -- the
published pitch is 5.41 m where the real one is 4.48 m, which would undercount a
35 m run by about one space.
"""
import argparse
import collections
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "vision"))
import score_occupancy as so                                    # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")

# Chaining rule. Deliberately geometric, and deliberately loose ALONG the curb and
# tight ACROSS it: consecutive bays are ~5.9 m apart (2.4 m where parking is
# perpendicular) and the gap of a driveway or a container bay should not end a run,
# while 2 m across is less than a lane, so the two sides of a street can never merge.
ALONG_MAX = 9.0      # m: largest gap along the curb that still continues a run
LATERAL_MAX = 2.0    # m: how far off the line a bay may sit and still belong
BEARING_COS = 0.95   # cos of the largest bearing disagreement (~18 deg)
MIN_RUN = 4          # bays: below this a "run" cannot establish an axis


def bay_bearing(f, ring):
    """Unit vector along the STREET at this bay.

    `make_bays.py` writes `bearing_deg` from the nearest OSM centerline, which is
    what we want: for perpendicular parking the bay's own long axis lies ACROSS the
    street, so taking the rectangle's long side would rotate the run by 90 degrees.
    Falls back to the rectangle's long axis only when the property is missing.
    """
    b = f["properties"].get("bearing_deg")
    if b is not None:
        return math.cos(math.radians(b)), math.sin(math.radians(b))
    best, bu = -1.0, (1.0, 0.0)
    for i in range(len(ring)):
        p, q = ring[i], ring[(i + 1) % len(ring)]
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d > best:
            best, bu = d, ((q[0] - p[0]) / d, (q[1] - p[1]) / d)
    return bu


def load_bays(path):
    feats = json.load(open(path, encoding="utf-8"))["features"]
    out = []
    for f in feats:
        ring = [so.to_enu(lon, lat)
                for lon, lat in f["geometry"]["coordinates"][0][:-1]]
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
        pr = f["properties"]
        out.append({"id": str(pr.get("id")), "cx": cx, "cy": cy,
                    "u": bay_bearing(f, ring),
                    "street": pr.get("mestopoloz"), "zona": pr.get("zona"),
                    "park_txt": pr.get("park_txt"), "vid": pr.get("vid_txt_20")})
    return out


def chain(bays):
    """Union-find over the chaining rule -> [[bay_index, ...], ...]."""
    parent = list(range(len(bays)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    # Bucket by a coarse grid so this stays near-linear instead of O(n^2):
    # a partner has to be within ALONG_MAX, so only neighbouring cells can match.
    cell = ALONG_MAX
    grid = collections.defaultdict(list)
    for i, b in enumerate(bays):
        grid[(int(b["cx"] // cell), int(b["cy"] // cell))].append(i)

    for i, a in enumerate(bays):
        gx, gy = int(a["cx"] // cell), int(a["cy"] // cell)
        ux, uy = a["u"]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((gx + dx, gy + dy), ()):
                    if j <= i:
                        continue
                    b = bays[j]
                    if abs(ux * b["u"][0] + uy * b["u"][1]) < BEARING_COS:
                        continue
                    vx, vy = b["cx"] - a["cx"], b["cy"] - a["cy"]
                    if abs(vx * ux + vy * uy) > ALONG_MAX:
                        continue
                    if abs(-vx * uy + vy * ux) > LATERAL_MAX:
                        continue
                    ra, rb = find(i), find(j)
                    if ra != rb:
                        parent[rb] = ra

    groups = collections.defaultdict(list)
    for i in range(len(bays)):
        groups[find(i)].append(i)
    return list(groups.values())


def principal_axis(pts):
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    # larger eigenvector of [[sxx, sxy], [sxy, syy]]
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    lam = tr / 2.0 + math.sqrt(max(0.0, tr * tr / 4.0 - det))
    vx, vy = (sxy, lam - sxx) if abs(sxy) > 1e-9 else (1.0, 0.0)
    d = math.hypot(vx, vy) or 1.0
    return (mx, my), (vx / d, vy / d)


def arclength(line):
    s = [0.0]
    for a, b in zip(line, line[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return s


def extend_ends(line, half):
    """Push each end out by `half` along its own terminal segment.

    A run's usable curb starts at the outer EDGE of its first bay, not at that
    bay's centre, so a polyline through the centroids is one bay short overall --
    which would show up directly as a missing free space at each end.
    """
    if len(line) < 2 or not half:
        return line
    out = list(line)
    for end, nxt in ((0, 1), (len(line) - 1, len(line) - 2)):
        ax, ay = line[end]
        bx, by = line[nxt]
        d = math.hypot(ax - bx, ay - by) or 1.0
        out[end] = (ax + (ax - bx) / d * half, ay + (ay - by) / d * half)
    return out


def build_run(run_id, bays, idx):
    """A chained group -> the run record. The polyline follows the ordered
    centroids, so a curving street stays curved instead of being straightened
    onto a single fitted line."""
    members = [bays[i] for i in idx]
    pts = [(b["cx"], b["cy"]) for b in members]
    if len(pts) > 1:
        (mx, my), u = principal_axis(pts)
    else:
        (mx, my), u = pts[0], members[0]["u"]
    order = sorted(range(len(members)),
                   key=lambda k: (pts[k][0] - mx) * u[0] + (pts[k][1] - my) * u[1])
    members = [members[k] for k in order]
    line = [pts[k] for k in order]

    s = arclength(line)
    gaps = sorted(b - a for a, b in zip(s, s[1:]))
    pitch = gaps[len(gaps) // 2] if gaps else None

    line = extend_ends(line, (pitch / 2.0) if pitch else 0.0)
    s = arclength(line)

    def majority(key):
        c = collections.Counter(m[key] for m in members if m[key])
        return c.most_common(1)[0][0] if c else None

    return {
        "run_id": run_id,
        "street": majority("street"),
        "zona": majority("zona"),
        "park_txt": majority("park_txt"),
        "capacity": len(members),          # Sofiaplan's count -- never length/pitch
        "pitch_m": round(pitch, 3) if pitch else None,
        "length_m": round(s[-1], 2),
        "bearing_deg": round(math.degrees(math.atan2(u[1], u[0])) % 180.0, 1),
        "verified": len(members) >= MIN_RUN,
        "bay_ids": [m["id"] for m in members],
        "line": line,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bays", default=os.path.join(DATA, "block_bays.geojson"))
    ap.add_argument("--out", default=os.path.join(DATA, "curb_runs.geojson"))
    ap.add_argument("--report", action="store_true", help="print only, write nothing")
    a = ap.parse_args()

    bays = load_bays(a.bays)
    groups = chain(bays)
    runs = [build_run(f"run_{i:04d}", bays, g)
            for i, g in enumerate(sorted(groups, key=lambda g: -len(g)))]

    big = [r for r in runs if r["verified"]]
    covered = sum(r["capacity"] for r in big)
    print(f"{len(bays)} bays -> {len(runs)} chains, "
          f"{len(big)} runs of >={MIN_RUN} covering {covered} bays "
          f"({100.0 * covered / len(bays):.0f}%)")
    if big:
        lens = sorted(r["length_m"] for r in big)
        caps = sorted(r["capacity"] for r in big)
        pitches = sorted(r["pitch_m"] for r in big if r["pitch_m"])
        print(f"  median run: {lens[len(lens) // 2]:.0f} m, "
              f"{caps[len(caps) // 2]} bays;  total curb "
              f"{sum(lens) / 1000.0:.2f} km")
        print(f"  median pitch {pitches[len(pitches) // 2]:.2f} m "
              f"(published rectangles are 5.40 m long)")
    unver = sum(r["capacity"] for r in runs if not r["verified"])
    print(f"  {unver} bays ({100.0 * unver / len(bays):.0f}%) are in chains of "
          f"<{MIN_RUN} -- carried as UNVERIFIED, never defaulted")

    for k, v in collections.Counter(r["park_txt"] for r in big).most_common():
        ps = sorted(r["pitch_m"] for r in big if r["park_txt"] == k and r["pitch_m"])
        tail = f", median pitch {ps[len(ps) // 2]:.2f} m" if ps else ""
        print(f"    {str(k):10s} {v:3d} runs{tail}")

    if a.report:
        return

    feats = []
    for r in runs:
        line = r.pop("line")
        feats.append({
            "type": "Feature",
            "properties": r,
            "geometry": {"type": "LineString",
                         "coordinates": [[round(so.ORIGIN[1] + x / so.MLON, 8),
                                          round(so.ORIGIN[0] + y / so.MLAT, 8)]
                                         for x, y in line]},
        })
    json.dump({"type": "FeatureCollection", "features": feats},
              open(a.out, "w", encoding="utf-8"), ensure_ascii=False)
    print(f"\nwrote {a.out}  ({len(feats)} runs)")


if __name__ == "__main__":
    main()
