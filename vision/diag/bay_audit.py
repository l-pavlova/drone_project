"""Audit Sofiaplan bay geometry against OSM -- a reference nobody here produced.

    python vision/diag/bay_audit.py                       # the 1km block
    python vision/diag/bay_audit.py --by-street --min 8   # per-street table
    python vision/diag/bay_audit.py --dets vision/runs/dji_0075_occupancy.json

The drone-side checks (`real_align.py`, `paint_ground_control.py`) can only ever
compare the bays with OUR pose chain. This one does not use a photograph at all:
it asks whether each bay is in a place a parking bay can physically be, using OSM
road centerlines and building footprints as the outside reference.

Four tests, in order of how hard they are to argue with:

  * **in a building.** A bay whose centroid lands inside a footprint is wrong, full
    stop -- and it is unscoreable by any camera, because the camera sees the roof.
  * **distance to the nearest road centerline.** Kerbside parking sits roughly
    (carriageway half-width + bay depth) from the centre of its street, so the
    healthy population is a tight band a few metres wide. A bay tens of metres out
    is in a courtyard, a garden, or on the wrong street.
  * **bearing against that road.** Sofiaplan records `park_txt` (Надлъжно =
    parallel, Напречно/Под ъгъл = perpendicular/angled), so the bay's own long
    axis should sit at a predictable angle to the street. A parallel bay 40 deg off
    its street is drawn wrong however close it is.
  * **name agreement.** The bay's `mestopoloz` against the nearest road's OSM
    `name`, through a copy of `generate_world.norm_street` (see below for why it is
    copied). A disagreement here usually means
    the bay was matched to the wrong street by the distance test, so it is read as
    a flag on the other three rather than as a finding of its own.

`--dets` runs the SAME distance-to-road test on detected cars from a real flight.
That is the comparison worth having: if the cars sit in the healthy kerbside band
and the bays that are supposed to hold them do not, the disagreement is the data's.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


import score_occupancy as so                                     # noqa: E402


# Mirrors sim/generate_world.py::norm_street -- NOT imported, because that module
# generates a world at import time (it is a script first) and would overwrite
# sim/worlds/*.wbt as a side effect of running this audit. Keep the two in step;
# same standing duplication as ORIGIN and the DS_* constants.
_ABBREV = {"арх.": "архитект", "св.": "свети", "проф.": "професор",
           "ген.": "генерал", "инж.": "инженер", "д-р": "доктор"}


def norm_street(name):
    if not name:
        return ""
    out = name.lower().strip()
    for pref in ("ул.", "бул.", "жк", "кв.", "пл."):
        if out.startswith(pref):
            out = out[len(pref):].strip()
            break
    for abbr, full in _ABBREV.items():
        out = out.replace(abbr, full)
    return " ".join(out.split())

sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data")


def seg_dist(px, py, ax, ay, bx, by):
    """Distance from P to segment AB, and the segment's bearing."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy), math.degrees(math.atan2(dy, dx)) % 180.0


def load_roads():
    """[(x0, y0, x1, y1, name), ...] in ENU metres."""
    segs = []
    for ft in json.load(open(os.path.join(DATA, "block_roads.geojson"),
                             encoding="utf-8"))["features"]:
        name = ft["properties"].get("name") or ""
        pts = [so.to_enu(c[0], c[1]) for c in ft["geometry"]["coordinates"]]
        for i in range(len(pts) - 1):
            segs.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], name))
    return segs


def load_buildings():
    polys = []
    for ft in json.load(open(os.path.join(DATA, "block_areas.geojson"),
                             encoding="utf-8"))["features"]:
        if ft["properties"].get("kind") != "building":
            continue
        polys.append([so.to_enu(c[0], c[1]) for c in ft["geometry"]["coordinates"][0]])
    return polys


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            inside = not inside
    return inside


class Grid:
    """Uniform bucket index -- 191 roads x 1698 bays brute force is 30M segment tests."""

    def __init__(self, items, cell, key):
        self.cell, self.b = cell, {}
        for it in items:
            for c in key(it):
                self.b.setdefault(c, []).append(it)

    def near(self, x, y, rings=1):
        cx, cy = int(x // self.cell), int(y // self.cell)
        out = []
        for i in range(cx - rings, cx + rings + 1):
            for j in range(cy - rings, cy + rings + 1):
                out.extend(self.b.get((i, j), ()))
        return out


def road_grid(segs, cell=40.0):
    def cells(s):
        x0, y0, x1, y1, _ = s
        out = set()
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) // (cell / 2)) + 1)
        for k in range(n + 1):
            t = k / n
            out.add((int((x0 + t * (x1 - x0)) // cell), int((y0 + t * (y1 - y0)) // cell)))
        return out
    return Grid(segs, cell, cells)


def nearest_road(gx, x, y):
    best = (1e9, 0.0, "")
    for rings in (1, 2, 4, 8):
        for s in gx.near(x, y, rings):
            d, brg = seg_dist(x, y, s[0], s[1], s[2], s[3])
            if d < best[0]:
                best = (d, brg, s[4])
        if best[0] < 1e9:
            break
    return best


def ring_axis(ring):
    """Long-axis bearing (deg, mod 180) and the long/short side lengths of a quad."""
    e = [(math.hypot(ring[i + 1][0] - ring[i][0], ring[i + 1][1] - ring[i][1]),
          math.degrees(math.atan2(ring[i + 1][1] - ring[i][1],
                                  ring[i + 1][0] - ring[i][0])) % 180.0)
         for i in range(len(ring) - 1)]
    e.sort(reverse=True)
    return e[0][1], e[0][0], e[-1][0]


def ang_diff(a, b):
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def pct(vals, p):
    if not vals:
        return float("nan")
    v = sorted(vals)
    return v[min(len(v) - 1, int(round(p / 100.0 * (len(v) - 1))))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bays", default=os.path.join(DATA, "block_bays.geojson"))
    ap.add_argument("--far", type=float, default=15.0,
                    help="centroid-to-centerline distance called implausible (m)")
    ap.add_argument("--skew", type=float, default=25.0,
                    help="bearing disagreement called implausible (deg)")
    ap.add_argument("--by-street", action="store_true")
    ap.add_argument("--min", type=int, default=10, help="min bays for a street row")
    ap.add_argument("--dets", help="also audit a flight's detections the same way")
    ap.add_argument("--json", help="write the per-bay table here")
    a = ap.parse_args()

    segs = load_roads()
    gx = road_grid(segs)
    blds = load_buildings()
    bg = Grid([(p, min(x for x, _ in p), min(y for _, y in p),
                max(x for x, _ in p), max(y for _, y in p)) for p in blds], 40.0,
              lambda it: {(i, j)
                          for i in range(int(it[1] // 40), int(it[3] // 40) + 1)
                          for j in range(int(it[2] // 40), int(it[4] // 40) + 1)})

    rows = []
    for ft in json.load(open(a.bays, encoding="utf-8"))["features"]:
        pr = ft["properties"]
        ring = [so.to_enu(c[0], c[1]) for c in ft["geometry"]["coordinates"][0]]
        cx = sum(p[0] for p in ring[:-1]) / (len(ring) - 1)
        cy = sum(p[1] for p in ring[:-1]) / (len(ring) - 1)
        d, rbrg, rname = nearest_road(gx, cx, cy)
        brg, long_m, short_m = ring_axis(ring)
        kind = (pr.get("park_txt") or "").strip()
        # Надлъжно = parallel to the kerb; anything else is across it.
        parallel = kind.startswith("Надлъжн")
        skew = ang_diff(brg, rbrg) if parallel else ang_diff(ang_diff(brg, rbrg), 90.0)
        rows.append(dict(
            id=pr.get("id"), street=pr.get("mestopoloz") or "", kind=kind,
            x=round(cx, 2), y=round(cy, 2), road_m=round(d, 2), road=rname,
            skew_deg=round(skew, 1), long_m=round(long_m, 2), short_m=round(short_m, 2),
            in_building=any(point_in_poly(cx, cy, it[0]) for it in bg.near(cx, cy)),
            name_ok=bool(rname) and (norm_street(pr.get("mestopoloz") or "") ==
                                     norm_street(rname)),
        ))

    n = len(rows)
    dists = [r["road_m"] for r in rows]
    inb = [r for r in rows if r["in_building"]]
    far = [r for r in rows if r["road_m"] > a.far]
    sk = [r for r in rows if r["skew_deg"] > a.skew]
    bad = [r for r in rows if r["in_building"] or r["road_m"] > a.far or r["skew_deg"] > a.skew]

    print(f"=== {n} bays vs OSM ===")
    print(f"distance to nearest road centerline: median {pct(dists, 50):.1f} m  "
          f"p10 {pct(dists, 10):.1f}  p90 {pct(dists, 90):.1f}  max {max(dists):.1f}")
    hist = [0] * 8
    for d in dists:
        hist[min(7, int(d // 5))] += 1
    for i, c in enumerate(hist):
        lo = i * 5
        lab = f"{lo:>2}-{lo + 5:<2}m" if i < 7 else "  35+m"
        print(f"  {lab} {c:>5}  {'#' * int(60 * c / n)}")
    print(f"centroid inside an OSM building : {len(inb):>5}  ({100 * len(inb) / n:.1f}%)")
    print(f"further than {a.far:.0f} m from any road: {len(far):>5}  ({100 * len(far) / n:.1f}%)")
    print(f"bearing off its street >{a.skew:.0f} deg  : {len(sk):>5}  ({100 * len(sk) / n:.1f}%)")
    print(f"FLAGGED by any test             : {len(bad):>5}  ({100 * len(bad) / n:.1f}%)")
    nm = [r for r in rows if not r["name_ok"]]
    print(f"(name of nearest road disagrees : {len(nm):>5}  -- see docstring)")

    if a.by_street:
        by = {}
        for r in rows:
            by.setdefault(r["street"], []).append(r)
        print(f"\n=== per street (>= {a.min} bays), worst first ===")
        print(f"{'bays':>5} {'med m':>6} {'p90 m':>6} {'skew':>5} {'bldg':>5} {'flag%':>6}  street")
        tab = []
        for s, rs in by.items():
            if len(rs) < a.min:
                continue
            f = sum(1 for r in rs if r["in_building"] or r["road_m"] > a.far
                    or r["skew_deg"] > a.skew)
            tab.append((f / len(rs), len(rs), pct([r["road_m"] for r in rs], 50),
                        pct([r["road_m"] for r in rs], 90),
                        pct([r["skew_deg"] for r in rs], 50),
                        sum(1 for r in rs if r["in_building"]), s))
        for fr, c, m, p9, sk50, nb, s in sorted(tab, reverse=True):
            print(f"{c:>5} {m:>6.1f} {p9:>6.1f} {sk50:>5.0f} {nb:>5} {100 * fr:>5.0f}%  {s}")

    if a.dets:
        res = json.load(open(a.dets, encoding="utf-8"))
        dd = [nearest_road(gx, p["x"], p["y"])[0] for p in res.get("unassigned", [])]
        if dd:
            print(f"\n=== {len(dd)} detected cars from {os.path.basename(a.dets)} ===")
            print(f"distance to nearest road centerline: median {pct(dd, 50):.1f} m  "
                  f"p10 {pct(dd, 10):.1f}  p90 {pct(dd, 90):.1f}")
            print("  (compare with the bay median above -- a car parks at the kerb, "
                  "so the two populations should sit in the same band)")

    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        json.dump(rows, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print(f"\nwrote {a.json}")


if __name__ == "__main__":
    main()
