#!/usr/bin/env python3
"""
Generate a Webots .wbt world from the FMI parking bays.

Reads ../data/block_bays.geojson, projects lon/lat to local metres (ENU, z up),
keeps a square window around the centre, and writes:
  worlds/fmi_block.wbt        - ground, painted bays, parked cars, Mavic2Pro drone
  worlds/ground_truth.json    - bay_id -> occupied (for detection evaluation)

Usage:
    python generate_world.py [window_half_m] [occupied_fraction] [world_name]
    python generate_world.py 75 0.5                    # default fmi_block.wbt
    python generate_world.py 500 0.5 fmi_block_1km     # separate big world:
        writes fmi_block_1km.wbt + fmi_block_1km.route.json +
        fmi_block_1km.ground_truth.json alongside the default world; the .wbt
        hands its route file to the controller via controllerArgs, so both
        worlds coexist and stay runnable (default keeps legacy route.json /
        ground_truth.json names).

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
NAME = sys.argv[3] if len(sys.argv) > 3 else "fmi_block"
# the default world keeps the legacy file names (controller falls back to them)
ROUTE_FILE = "route.json" if NAME == "fmi_block" else f"{NAME}.route.json"
GT_FILE = "ground_truth.json" if NAME == "fmi_block" else f"{NAME}.ground_truth.json"
WEBOTS_VER = "R2023b"
random.seed(42)

feats = json.load(open(BAYS, encoding="utf-8"))["features"]

# The project's shared georeference origin: every world, route, pose and
# ground truth uses ENU metres about THIS lat/lon. Pinned (= the bay centroid
# of the original 250 m FMI-block cut) so re-cutting the data at a bigger
# half-size doesn't shift the frame of already-flown worlds; change it
# deliberately only if the study area moves.
ORIGIN = (42.6747105, 23.3298956)   # lat, lon

def ring_center(f):
    r = f["geometry"]["coordinates"][0][:-1]
    return sum(p[0] for p in r) / len(r), sum(p[1] for p in r) / len(r)

centers = [ring_center(f) for f in feats]
lat0, lon0 = ORIGIN
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
# Flight route: POSTMAN (route-inspection) walk of the street network. The
# coverage graph is the OSM centerlines (block_roads.geojson, projected with
# the same lon0/lat0) of every street that has bays. Rural-postman recipe:
#   1. connect disconnected coverage components with their shortest road
#      transits (MST over components, Dijkstra on the full road graph);
#   2. every node of odd degree forces a repeat somewhere - pair the odd nodes
#      up with a minimum-weight matching (shortest road paths as pair costs)
#      and duplicate the matched paths; two virtual zero-cost endpoints let
#      the cheapest two odd nodes stay unmatched and become start/finish, so
#      the result is an OPEN path (no return to start);
#   3. the multigraph is now Eulerian: a Hierholzer walk flies every coverage
#      edge exactly once and deadheads only along the matched repeats.
# Deadheads and transits stay on the roads. Streets matching no OSM road fall
# back to a PCA line fit through their bay row. Route is written in the SAME
# local metres as the world so poses.json / ground_truth.json all share one
# coordinate frame.
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


def dijkstra(adj, srcs):
    """Multi-source Dijkstra; srcs is an iterable of nodes."""
    dist = {s: 0.0 for s in srcs if s in adj}
    prev, pq = {}, [(0.0, s) for s in dist]
    heapq.heapify(pq)
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


def min_matching(C):
    """Indices 0..n-1 (n even) paired to minimise sum of C[i][j]: exact
    blossom via networkx when installed, else exact bitmask DP for small n,
    else greedy + pair-swap refinement."""
    n = len(C)
    if n == 0:
        return []
    try:
        import networkx as nx
        G = nx.Graph()
        G.add_nodes_from(range(n))
        for i in range(n):
            for j in range(i + 1, n):
                G.add_edge(i, j, weight=C[i][j])
        return [tuple(e) for e in nx.min_weight_matching(G)]
    except ImportError:
        pass
    if n <= 14:
        INF = float("inf")
        dp = [INF] * (1 << n)
        dp[0] = 0.0
        choice = [None] * (1 << n)
        for m in range(1 << n):
            if dp[m] == INF:
                continue
            i = next(b for b in range(n) if not m & (1 << b))
            for j in range(i + 1, n):
                if m & (1 << j):
                    continue
                m2 = m | (1 << i) | (1 << j)
                nd = dp[m] + C[i][j]
                if nd < dp[m2]:
                    dp[m2] = nd
                    choice[m2] = (i, j)
        pairs, m = [], (1 << n) - 1
        while m:
            i, j = choice[m]
            pairs.append((i, j))
            m &= ~(1 << i) & ~(1 << j)
        return pairs
    # greedy: cheapest available pair first
    order = sorted((C[i][j], i, j) for i in range(n) for j in range(i + 1, n))
    free, pairs = set(range(n)), []
    for c, i, j in order:
        if i in free and j in free:
            pairs.append((i, j))
            free -= {i, j}
    # refinement: re-pair any two pairs if a swap is cheaper
    improved = True
    while improved:
        improved = False
        for a in range(len(pairs)):
            for b in range(a + 1, len(pairs)):
                i, j = pairs[a]
                k, l = pairs[b]
                cur = C[i][j] + C[k][l]
                for p, q in (((i, k), (j, l)), ((i, l), (j, k))):
                    alt = C[p[0]][p[1]] + C[q[0]][q[1]]
                    if alt < cur - 1e-9:
                        pairs[a], pairs[b] = p, q
                        cur = alt
                        improved = True
    return pairs


def path_back(prev, v):
    out = [v]
    while v in prev:
        v = prev[v]
        out.append(v)
    return out[::-1]


def build_route(bays, road_runs):
    """Open postman walk of the bay-streets' centerline graph: every coverage
    edge flown once, minimum-matched deadheads along the roads.
    road_runs: (osm_name, polyline) in local metres."""
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

    # ---- multigraph the drone will traverse: list of (u, v, node path u->v)
    walk_edges = [(a, b, [a, b]) for a in cov for b in cov[a] if a < b]
    cover_len = sum(cov[a][b] for a, b, _ in walk_edges)

    # ---- 1. connect coverage components with shortest road transits (MST)
    comps, seen = [], set()
    for n in cov:
        if n in seen:
            continue
        comp, stack = set(), [n]
        while stack:
            u = stack.pop()
            if u not in comp:
                comp.add(u)
                stack.extend(cov[u])
        seen |= comp
        comps.append(comp)
    tree = comps[0]
    rest = comps[1:]
    while rest:
        dist, prev = dijkstra(full, tree)
        best = min(((min((dist.get(v, 1e18), v) for v in comp), ci)
                    for ci, comp in enumerate(rest)))
        (d, node), ci = best
        if d < 1e17:
            path = path_back(prev, node)
        else:   # roads don't reach it: straight hop as a last resort
            path = [min(tree, key=lambda t: math.hypot(t[0] - node[0],
                                                       t[1] - node[1])), node]
            graph_add(full, path)
        walk_edges.append((path[0], path[-1], path))
        tree = tree | rest.pop(ci) | set(path)

    # ---- 2. even out odd-degree nodes: min-weight matching, open-path style
    deg = {}
    for u, v, _ in walk_edges:
        deg[u] = deg.get(u, 0) + 1
        deg[v] = deg.get(v, 0) + 1
    odd = [n for n, d in deg.items() if d % 2]
    sp = {n: dijkstra(full, [n]) for n in odd}     # dist+prev per odd node
    BIG = 1e15
    n_odd = len(odd)
    # two virtual endpoints (indices n_odd, n_odd+1): free to pair with any
    # odd node (those become start/finish), forbidden to pair with each other
    C = [[BIG] * (n_odd + 2) for _ in range(n_odd + 2)]
    for i in range(n_odd):
        for j in range(i + 1, n_odd):
            C[i][j] = C[j][i] = sp[odd[i]][0].get(odd[j], BIG)
        C[i][n_odd] = C[n_odd][i] = 0.0
        C[i][n_odd + 1] = C[n_odd + 1][i] = 0.0
    ends = []
    for i, j in min_matching(C):
        if i > j:
            i, j = j, i
        if j >= n_odd:                             # matched to a virtual
            ends.append(odd[i])
            continue
        dist_i, prev_i = sp[odd[i]]
        path = path_back(prev_i, odd[j])           # duplicated deadhead
        walk_edges.append((path[0], path[-1], path))

    # ---- 3. Hierholzer Euler walk over the multigraph
    incid = {}
    for ei, (u, v, _) in enumerate(walk_edges):
        incid.setdefault(u, []).append(ei)
        incid.setdefault(v, []).append(ei)
    # fly from whichever endpoint is nearer the drone's takeoff at the origin
    cands = ends if ends else list(incid)
    start = min(cands, key=lambda p: math.hypot(p[0], p[1]))
    used = [False] * len(walk_edges)
    trail, stack = [], [(start, None)]             # (node, edge arrived by)
    while stack:
        u, ein = stack[-1]
        nxt = None
        while incid.get(u):
            ei = incid[u][-1]
            if used[ei]:
                incid[u].pop()
                continue
            nxt = ei
            break
        if nxt is None:
            stack.pop()
            trail.append((u, ein))
        else:
            used[nxt] = True
            eu, ev, _ = walk_edges[nxt]
            stack.append((ev if u == eu else eu, nxt))
    trail.reverse()

    # expand each edge into its node path, oriented from the current node
    route = [trail[0][0]]
    for node, ein in trail[1:]:
        p = walk_edges[ein][2]
        route.extend(p[1:] if p[-1] == node else p[-2::-1])
    route_len = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(route, route[1:]))
    print(f"route: cover {cover_len:.0f} m + deadhead {route_len - cover_len:.0f} m "
          f"= {route_len:.0f} m")

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
DirectionalLight { direction 0.4 0.5 -1 intensity 2.5 castShadows FALSE }""")
GROUND = 2 * WINDOW + 200      # ground plane comfortably past the window
parts.append(f"""Solid {{
  name "ground"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor 0.32 0.33 0.34 roughness 1 metalness 0 }}
    geometry Plane {{ size {GROUND:.0f} {GROUND:.0f} }}
  }} ]
  boundingObject Plane {{ size {GROUND:.0f} {GROUND:.0f} }}
}}""")

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

# Bays look like real Sofia street parking: an asphalt pad with a thin white
# painted OUTLINE (~12 cm lines), not a filled white rectangle. The old filled
# rectangle made free/occupied trivially separable by colour (and made white
# cars invisible on it) — unrealistic in both directions.
LINE_W = 0.12                 # painted line width (m)
BAY_ASPHALT = "0.20 0.20 0.21"

def bay_marking(b):
    """Asphalt pad + 4 white outline lines for one bay, as Solid strings."""
    ca, sa = math.cos(b["ang"]), math.sin(b["ang"])
    out = [solid(b["x"], b["y"], 0.05, b["ang"], b["L"], b["W"], 0.02,
                 BAY_ASPHALT, name=f"bay_{b['id']}")]
    # (u, v) = centre of each line in the bay frame (u along length, v across),
    # with the line's box size; lines sit just above the pad
    lines = [(0, +(b["W"] - LINE_W) / 2, b["L"], LINE_W),
             (0, -(b["W"] - LINE_W) / 2, b["L"], LINE_W),
             (+(b["L"] - LINE_W) / 2, 0, LINE_W, b["W"]),
             (-(b["L"] - LINE_W) / 2, 0, LINE_W, b["W"])]
    for i, (u, v, sx, sy) in enumerate(lines):
        out.append(solid(b["x"] + u * ca - v * sa, b["y"] + u * sa + v * ca,
                         0.065, b["ang"], sx, sy, 0.012,
                         "0.95 0.95 0.95", name=f"bay_{b['id']}_l{i}"))
    return out

placed = []   # body rectangles (with clearance) of cars already placed
for b in bays:
    parts.extend(bay_marking(b))
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

# drone at centre (Mavic2Pro ships a gimbal camera named "camera");
# controllerArgs tells parkdrone which route file belongs to THIS world
parts.append(f"""Mavic2Pro {{
  translation 0 0 0.15
  controller "parkdrone"
  controllerArgs [ "{ROUTE_FILE}" ]
}}""")

route = build_route(bays, road_runs)

wbt = os.path.join(WORLDS, f"{NAME}.wbt")
open(wbt, "w", encoding="utf-8").write("\n".join(parts) + "\n")
json.dump(gt, open(os.path.join(WORLDS, GT_FILE), "w"), indent=0)
json.dump([[round(x, 2), round(y, 2)] for x, y in route],
          open(os.path.join(WORLDS, ROUTE_FILE), "w"), indent=0)

occ = sum(1 for b in bays if b["occupied"])
nstreets = len({b["street"] for b in bays})
print(f"window: {2*WINDOW:.0f}x{2*WINDOW:.0f} m   bays: {len(bays)}   "
      f"parked cars: {occ}   free public: {sum(1 for b in bays if b['public'] and not b['occupied'])}")
print(f"route: {len(route)} waypoints along {nstreets} street(s)")
print(f"wrote {wbt}")
print(f"wrote {os.path.join(WORLDS, GT_FILE)}")
print(f"wrote {os.path.join(WORLDS, ROUTE_FILE)}")
