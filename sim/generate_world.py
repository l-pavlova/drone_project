#!/usr/bin/env python3
"""
Generate a Webots .wbt world from the FMI parking bays.

Reads ../data/block_bays.geojson, projects lon/lat to local metres (ENU, z up),
keeps a square window around the centre, and writes:
  worlds/fmi_block.wbt        - ground, painted bays, parked cars, Mavic2Pro drone
  worlds/ground_truth.json    - bay_id -> occupied (for detection evaluation)

Usage:
    python generate_world.py [window_half_m] [occupied_fraction]
    python generate_world.py 75 0.5      # 150x150 m window, ~50% occupied

NOTE: not run/verified here — needs Webots installed to open. The Mavic2Pro proto
is pulled via EXTERNPROTO pinned to R2023b; if your Webots differs, change WEBOTS_VER.
"""
import json, os, math, sys, random, heapq

HERE = os.path.dirname(__file__)
BAYS = os.path.join(HERE, "..", "data", "block_bays.geojson")
ROADS = os.path.join(HERE, "..", "data", "block_roads.geojson")
WORLDS = os.path.join(HERE, "worlds")
os.makedirs(WORLDS, exist_ok=True)

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 75.0       # half-size, metres
OCC = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
WEBOTS_VER = "R2023b"
random.seed(42)

feats = json.load(open(BAYS, encoding="utf-8"))["features"]

# bay centres in lon/lat, then local metres about centroid
def ring_center(f):
    r = f["geometry"]["coordinates"][0][:-1]
    return sum(p[0] for p in r) / len(r), sum(p[1] for p in r) / len(r)

centers = [ring_center(f) for f in feats]
lon0 = sum(c[0] for c in centers) / len(centers)
lat0 = sum(c[1] for c in centers) / len(centers)
mlat = 111320.0
mlon = 111320.0 * math.cos(math.radians(lat0))

CAR_COLORS = ["0.8 0.1 0.1", "0.1 0.1 0.8", "0.9 0.9 0.9", "0.1 0.1 0.1",
              "0.6 0.6 0.6", "0.2 0.5 0.2"]

# Webots vehicle protos (static "Simple" variants, no physics). The Sprinter van
# is ~5.9 m long, so it only goes in longitudinal bays (L=5.4) and rarely.
CAR_PROTOS = {
    "TeslaModel3Simple":          "tesla/TeslaModel3Simple",
    "BmwX5Simple":                "bmw/BmwX5Simple",
    "CitroenCZeroSimple":         "citroen/CitroenCZeroSimple",
    "ToyotaPriusSimple":          "toyota/ToyotaPriusSimple",
    "LincolnMKZSimple":           "lincoln/LincolnMKZSimple",
    "RangeRoverSportSVRSimple":   "range_rover/RangeRoverSportSVRSimple",
    "MercedesBenzSprinterSimple": "mercedes_benz/MercedesBenzSprinterSimple",
}
CAR_MODELS = [m for m in CAR_PROTOS if m != "MercedesBenzSprinterSimple"]
VAN_MODEL = "MercedesBenzSprinterSimple"

# (length, width, centre_offset) in metres. Webots vehicle protos have their
# origin at the REAR AXLE, not the body centre; centre_offset is how far the
# body centre sits AHEAD of the origin, so the car can be centred in its bay.
CAR_DIMS = {
    "TeslaModel3Simple":          (4.69, 1.85, 1.38),
    "BmwX5Simple":                (4.89, 1.94, 1.45),
    "CitroenCZeroSimple":         (3.48, 1.48, 1.32),
    "ToyotaPriusSimple":          (4.54, 1.76, 1.38),
    "LincolnMKZSimple":           (4.93, 1.86, 1.34),
    "RangeRoverSportSVRSimple":   (4.85, 1.98, 1.40),
    "MercedesBenzSprinterSimple": (5.93, 1.99, 1.71),
}
CAR_GAP = 0.35   # min bumper-to-bumper clearance between parked cars (m)

def rect_corners(cx, cy, ang, L, W):
    c, s = math.cos(ang), math.sin(ang)
    return [(cx + c*dx - s*dy, cy + s*dx + c*dy)
            for dx, dy in ((L/2, W/2), (L/2, -W/2), (-L/2, -W/2), (-L/2, W/2))]

def rects_overlap(A, B):
    """Separating-axis test for two convex quads."""
    for quad in (A, B):
        for i in range(4):
            ex = quad[(i+1) % 4][0] - quad[i][0]
            ey = quad[(i+1) % 4][1] - quad[i][1]
            ax, ay = -ey, ex
            pa = [x*ax + y*ay for x, y in A]
            pb = [x*ax + y*ay for x, y in B]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True

bays = []
for f, (clon, clat) in zip(feats, centers):
    x = (clon - lon0) * mlon
    y = (clat - lat0) * mlat
    if abs(x) > WINDOW or abs(y) > WINDOW:
        continue
    pr = f["properties"]
    brg = math.radians(pr.get("bearing_deg", 0.0))
    if pr.get("park_txt") == "Напречн":
        ang = brg + math.pi / 2; L, W = 4.8, 2.4
    else:
        ang = brg; L, W = 5.4, 2.2
    bays.append({"id": pr.get("id"), "x": x, "y": y, "ang": ang, "L": L, "W": W,
                 "street": pr.get("mestopoloz"), "zona": pr.get("zona"),
                 "public": pr.get("vid_txt_20") == "Зона"})

# occupancy ground truth (only public bays can be 'occupied by a parked car')
gt = {}
for b in bays:
    b["occupied"] = (b["public"] and random.random() < OCC)
    gt[str(b["id"])] = b["occupied"]

# ---------------------------------------------------------------------------
# Flight route: DEPTH-FIRST walk of the street network. The coverage graph is
# the OSM centerlines (block_roads.geojson, projected with the same lon0/lat0)
# of every street that has bays; DFS starts at the westernmost street end and
# explores the shortest branch first at each junction, so side streets are
# covered and backtracked at their intersection before continuing along the
# main street (e.g. Bourchier west end -> intersection -> Sveta Gora and back
# -> Bourchier east end). Backtracks and transits stay on the roads. Streets
# matching no OSM road fall back to a PCA line fit through their bay row.
# Route is written in the SAME local metres as the world so poses.json /
# ground_truth.json all share one coordinate frame.
ROUTE_STEP = 10.0     # waypoint spacing (m); camera footprint at 30 m alt is only
                      # ~15 m along-track, and captures scatter a few m around
                      # each waypoint (orbit-timeout arrivals), so keep overlap
ROUTE_EXTEND = 8.0    # PCA fallback only: overshoot past the end bays (m)
MIN_BAYS_PER_STREET = 2

def fit_axis(pts):
    """Centroid + unit principal-axis direction (PCA) of a set of (x,y) points."""
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - cx) ** 2 for p in pts)
    syy = sum((p[1] - cy) ** 2 for p in pts)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in pts)
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return (cx, cy), (math.cos(theta), math.sin(theta))

def street_segment(pts):
    """Endpoints of the street row: project bays onto the fitted axis, take the
    min/max projection, extend a little past the end bays."""
    (cx, cy), (dx, dy) = fit_axis(pts)
    ts = [(p[0] - cx) * dx + (p[1] - cy) * dy for p in pts]
    tmin, tmax = min(ts) - ROUTE_EXTEND, max(ts) + ROUTE_EXTEND
    a = (cx + tmin * dx, cy + tmin * dy)
    b = (cx + tmax * dx, cy + tmax * dy)
    return a, b

def densify(a, b, step):
    """Waypoints from a to b (inclusive) spaced ~step metres apart."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    nseg = max(1, round(length / step))
    return [(a[0] + (b[0] - a[0]) * i / nseg,
             a[1] + (b[1] - a[1]) * i / nseg) for i in range(nseg + 1)]

def _key(p):
    """Graph node: vertex rounded to 0.5 m so shared OSM nodes merge."""
    return (round(p[0] * 2) / 2.0, round(p[1] * 2) / 2.0)


def graph_add(adj, run):
    ks = [_key(p) for p in run]
    for a, b in zip(ks, ks[1:]):
        if a == b:
            continue
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < adj.setdefault(a, {}).get(b, 1e18):
            adj[a][b] = d
            adj.setdefault(b, {})[a] = d


def dijkstra(adj, src):
    dist, prev, pq = {src: 0.0}, {}, [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, 1e18):
            continue
        for v, w in adj.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, 1e18):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, prev


def path_back(prev, v):
    out = [v]
    while v in prev:
        v = prev[v]
        out.append(v)
    return out[::-1]


def build_route(bays, road_runs):
    """Depth-first walk of the bay-streets' centerline graph, starting at the
    westernmost street end. road_runs: (osm_name, polyline) in local metres."""
    streets = {}
    for b in bays:
        streets.setdefault(b["street"], []).append((b["x"], b["y"]))

    # coverage targets: the in-window portion of each bay-street's centerline
    targets = []
    for name, pts in streets.items():
        if len(pts) < MIN_BAYS_PER_STREET:
            continue
        cov = []
        for rname, run in road_runs:
            if rname and name and (rname in name or name in rname):
                cov.extend(clip_polyline(run, WINDOW + 5.0))
        if not cov:   # no OSM road matched this street name: PCA bay-row fit
            cov = [list(street_segment(pts))]
        targets.extend(cov)

    # coverage graph (edges to fly) + full road graph (for transits between
    # disconnected street groups)
    cov = {}
    for t in targets:
        graph_add(cov, t)
    full = {}
    for _, run in road_runs:
        graph_add(full, run)
    for t in targets:
        graph_add(full, t)

    unvisited = {(min(a, b), max(a, b)) for a in cov for b in cov[a]}

    def subtree_len(u, v, seen):
        """Total unvisited coverage length reachable by entering edge u->v."""
        e = (min(u, v), max(u, v))
        if e in seen or e not in unvisited:
            return 0.0
        seen.add(e)
        return cov[u][v] + sum(subtree_len(v, w, seen) for w in cov[v])

    route = []
    last_new = [0]   # route index right after the most recent NEW edge

    def dfs(u):
        while True:
            nbrs = [v for v in cov[u] if (min(u, v), max(u, v)) in unvisited]
            if not nbrs:
                return
            # shortest branch first: dead-end side streets get covered and
            # backtracked before we continue down the main street
            v = min(nbrs, key=lambda w: subtree_len(u, w, set()))
            unvisited.discard((min(u, v), max(u, v)))
            route.append(v)
            last_new[0] = len(route)
            dfs(v)
            route.append(u)   # backtrack along the street

    def endpoints(adjacency, edges):
        nodes = {n for e in edges for n in e}
        return [n for n in nodes
                if sum((min(n, v), max(n, v)) in edges for v in adjacency[n]) <= 1] or list(nodes)

    root = min(endpoints(cov, unvisited))          # westernmost street end
    route.append(root)
    dfs(root)
    while unvisited:                               # disconnected street group
        cur = route[last_new[0] - 1]
        dist, prev = dijkstra(full, cur) if cur in full else ({}, {})
        nxt = min(endpoints(cov, unvisited),
                  key=lambda n: dist.get(n, 3.0 * math.hypot(n[0] - cur[0],
                                                             n[1] - cur[1])))
        del route[last_new[0]:]                    # drop the trailing backtrack
        if nxt in dist:
            route.extend(path_back(prev, nxt)[1:]) # transit along the streets
        else:
            route.append(nxt)
        last_new[0] = len(route)
        dfs(nxt)
    del route[last_new[0]:]                        # don't backtrack at the end

    # drop near-duplicate consecutive points, then densify into waypoints
    pts = [route[0]]
    for p in route[1:]:
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 0.5:
            pts.append(p)
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        out.extend(densify(a, b, ROUTE_STEP)[1:])
    return out


def solid(x, y, z, ang, sx, sy, sz, color, name, rough="1", metal="0"):
    return f"""Solid {{
  translation {x:.3f} {y:.3f} {z:.3f}
  rotation 0 0 1 {ang:.4f}
  name "{name}"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color} roughness {rough} metalness {metal} }}
    geometry Box {{ size {sx:.3f} {sy:.3f} {sz:.3f} }}
  }} ]
}}"""

GH = f"https://raw.githubusercontent.com/cyberbotics/webots/{WEBOTS_VER}/projects"
parts = []
parts.append(f"#VRML_SIM {WEBOTS_VER} utf8")
parts.append(f'EXTERNPROTO "{GH}/robots/dji/mavic/protos/Mavic2Pro.proto"')
parts.append(f'EXTERNPROTO "{GH}/objects/road/protos/Road.proto"')
parts.append(f'EXTERNPROTO "{GH}/objects/road/protos/RoadLine.proto"')
for path in CAR_PROTOS.values():
    parts.append(f'EXTERNPROTO "{GH}/vehicles/protos/{path}.proto"')
parts.append("""WorldInfo { basicTimeStep 8 }
Viewpoint {
  orientation -0.35 0.35 0.87 1.75
  position -8 -14 10
  follow "Mavic 2 PRO"
  followType "Tracking Shot"
}
Background { skyColor [ 0.5 0.7 1 ] }
DirectionalLight { direction 0.4 0.5 -1 intensity 2.5 castShadows FALSE }
Solid {
  name "ground"
  children [ Shape {
    appearance PBRAppearance { baseColor 0.32 0.33 0.34 roughness 1 metalness 0 }
    geometry Plane { size 600 600 }
  } ]
  boundingObject Plane { size 600 600 }
}""")

# --- streets: OSM centerlines rendered as Road protos (asphalt + dashed line).
# Same lon0/lat0 projection as the bays; no bounding objects, so physics is
# untouched. Ways are clipped to the window (+margin) and split into runs.
ROAD_MARGIN = 15.0

def road_width(pr):
    try:
        return max(5.0, 3.25 * int(pr.get("lanes")))
    except (TypeError, ValueError):
        pass
    return 9.0 if pr.get("highway") in ("primary", "secondary", "tertiary") else 6.5

def clip_segment(p, q, lim):
    """Liang-Barsky: part of segment p-q inside |x|,|y| <= lim, or None.
    OSM nodes are sparse (often only at intersections), so segments must be
    clipped - filtering points would drop streets that cross the window."""
    t0, t1 = 0.0, 1.0
    dx, dy = q[0] - p[0], q[1] - p[1]
    for d, lo in ((dx, p[0]), (-dx, -p[0]), (dy, p[1]), (-dy, -p[1])):
        # d*t + lo <= lim  (right, left, top, bottom in turn)
        if d == 0:
            if lo > lim:
                return None
        else:
            t = (lim - lo) / d
            if d > 0:
                t1 = min(t1, t)
            else:
                t0 = max(t0, t)
    if t0 >= t1:
        return None
    return ((p[0] + t0 * dx, p[1] + t0 * dy), (p[0] + t1 * dx, p[1] + t1 * dy))

def clip_polyline(pts, lim):
    """Split a polyline into the runs inside |x|,|y| <= lim (segment-clipped)."""
    runs, run = [], []
    for p, q in zip(pts, pts[1:]):
        seg = clip_segment(p, q, lim)
        if seg is None:
            if run:
                runs.append(run); run = []
            continue
        a, b = seg
        if not run:
            run = [a]
        elif math.hypot(a[0] - run[-1][0], a[1] - run[-1][1]) > 0.01:
            runs.append(run); run = [a]   # re-entered the window
        run.append(b)
    if run:
        runs.append(run)
    return [r for r in runs if len(r) >= 2]

road_runs = []   # (osm name, projected polyline) - also feeds the route builder
n_roads = 0
if os.path.exists(ROADS):
    lim = WINDOW + ROAD_MARGIN
    for f in json.load(open(ROADS, encoding="utf-8"))["features"]:
        pts = [((lon - lon0) * mlon, (lat - lat0) * mlat)
               for lon, lat in f["geometry"]["coordinates"]]
        pr = f["properties"]
        for run in clip_polyline(pts, lim):
            road_runs.append((pr.get("name") or "", run))
            wpts = ", ".join(f"{x:.2f} {y:.2f} 0" for x, y in run)
            # stagger heights so overlapping roads (intersections) don't z-fight
            parts.append(f"""Road {{
  translation 0 0 {0.01 + 0.003 * n_roads:.3f}
  name "road_{n_roads} {pr.get('name') or pr.get('highway')}"
  width {road_width(pr):.2f}
  numberOfLanes 2
  lines [ RoadLine {{ type "dashed" }} ]
  roadBorderHeight 0
  rightBorder FALSE
  leftBorder FALSE
  wayPoints [ {wpts} ]
}}""")
            n_roads += 1
else:
    print("WARNING: no block_roads.geojson (run tools/get_roads.py) - world has no streets")

rng_cars = random.Random(7)   # separate stream so ground_truth.json stays stable

placed = []   # body rectangles (with clearance) of cars already placed
for b in bays:
    # painted bay marking (white, thin); sits above the staggered road surfaces
    parts.append(solid(b["x"], b["y"], 0.05, b["ang"], b["L"], b["W"], 0.02,
                       "0.95 0.95 0.95", name=f"bay_{b['id']}"))
    if b["occupied"]:
        longitudinal = b["L"] > 5.0
        if longitudinal and rng_cars.random() < 0.12:
            want = VAN_MODEL   # van only fits (barely) along-street bays
        else:
            want = rng_cars.choice(CAR_MODELS)
        col = rng_cars.choice(CAR_COLORS)
        ang = b["ang"] + (0.0 if rng_cars.random() < 0.5 else math.pi)  # nose either way
        # Some bay rows sit tighter than the bay paint (source points ~4.5 m
        # apart on Bourchier), so try the wanted model first, then shorter and
        # shorter ones, and leave the bay free if even the smallest car clips
        # an already-placed neighbour.
        model = None
        for cand in [want] + sorted(CAR_MODELS, key=lambda m: CAR_DIMS[m][0]):
            L, W, _ = CAR_DIMS[cand]
            rect = rect_corners(b["x"], b["y"], ang, L + CAR_GAP, W)
            if not any(rects_overlap(rect, r) for r in placed):
                model = cand
                break
        if model is None:
            b["occupied"] = False
            gt[str(b["id"])] = False
            continue
        L, W, off = CAR_DIMS[model]
        placed.append(rect_corners(b["x"], b["y"], ang, L + CAR_GAP, W))
        # proto origin is the rear axle: pull it back so the BODY is bay-centred
        tx = b["x"] - math.cos(ang) * off
        ty = b["y"] - math.sin(ang) * off
        parts.append(f"""{model} {{
  translation {tx:.3f} {ty:.3f} 0.4
  rotation 0 0 1 {ang:.4f}
  color {col}
  name "car_{b['id']}"
}}""")

# drone at centre (Mavic2Pro ships a gimbal camera named "camera")
parts.append("""Mavic2Pro {
  translation 0 0 0.15
  controller "parkdrone"
}""")

route = build_route(bays, road_runs)

wbt = os.path.join(WORLDS, "fmi_block.wbt")
open(wbt, "w", encoding="utf-8").write("\n".join(parts) + "\n")
json.dump(gt, open(os.path.join(WORLDS, "ground_truth.json"), "w"), indent=0)
json.dump([[round(x, 2), round(y, 2)] for x, y in route],
          open(os.path.join(WORLDS, "route.json"), "w"), indent=0)

occ = sum(1 for b in bays if b["occupied"])
nstreets = len({b["street"] for b in bays})
print(f"window: {2*WINDOW:.0f}x{2*WINDOW:.0f} m   bays: {len(bays)}   "
      f"parked cars: {occ}   free public: {sum(1 for b in bays if b['public'] and not b['occupied'])}")
print(f"route: {len(route)} waypoints along {nstreets} street(s)")
print(f"wrote {wbt}")
print(f"wrote {os.path.join(WORLDS, 'ground_truth.json')}")
print(f"wrote {os.path.join(WORLDS, 'route.json')}")
