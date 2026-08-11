#!/usr/bin/env python3
"""
Generate a Webots .wbt world from the FMI parking bays.

Reads ../data/block_bays.geojson, projects lon/lat to local metres (ENU, z up),
keeps a square window around the centre, and writes:
  worlds/fmi_block.wbt        - ground, painted bays, parked cars, Mavic2Pro drone
  worlds/ground_truth.json    - bay_id -> occupied (for detection evaluation)

Usage:
    python generate_world.py [window_half_m] [occupied_fraction] [survey_area] [--collide] [--chase] [--obstacles]
    python generate_world.py 75 0.5                    # default fmi_block.wbt
    python generate_world.py 500 0.5 fmi_block_1km     # separate big world:
        writes fmi_block_1km.wbt + fmi_block_1km.route.json +
        fmi_block_1km.ground_truth.json alongside the default world; the .wbt
        hands its route file to the controller via controllerArgs, so both
        worlds coexist and stay runnable (default keeps legacy route.json /
        ground_truth.json names).
    python generate_world.py 130 0.5 fmi_block_obst --collide --obstacles --chase
        # the obstacle-avoidance TEST world (see --obstacles below)

--collide gives collision geometry to the scenery: a bounding object on the
buildings AND on the generated light poles (a Cylinder matching the mast). OFF
by default: the controller flies a fixed 30 m with no obstacle logic at all, so
collision geometry would crash the drone on any tall block under the route (and
burn the >1 h 1 km patrol). Webots range sensors only see nodes that HAVE a
bounding object, so the obstacle-avoidance work turns this on and regenerates.

--obstacles builds an OBSTACLE COURSE: a handful of synthetic structures parked
ON the flight route, tall enough to reach the 30 m cruise altitude. It exists
because the only real world with anything to hit is fmi_block_1km, which takes
>10 min to load - so the avoidance edit/run/observe loop needs a small world
with a deliberate conflict. The course is deterministic (anchored to fractions
along the route, nudged forward off any bay it would cover) and the structures
are appended to <NAME>.hazards.json alongside the real ones, flagged
"synthetic": true. It changes nothing unless the flag is passed, so the three
survey worlds stay byte-identical. Use --collide with it or the drone flies
straight through the course and the sensors see nothing.

NOTE: not run/verified here — needs Webots installed to open. The Mavic2Pro proto
is pulled via EXTERNPROTO pinned to R2023b; if your Webots differs, change WEBOTS_VER.
"""
import json, os, math, sys, random, heapq

HERE = os.path.dirname(__file__)
BAYS = os.path.join(HERE, "..", "data", "block_bays.geojson")
ROADS = os.path.join(HERE, "..", "data", "block_roads.geojson")
AREAS = os.path.join(HERE, "..", "data", "block_areas.geojson")
WORLDS = os.path.join(HERE, "worlds")
os.makedirs(WORLDS, exist_ok=True)

args = [a for a in sys.argv[1:] if not a.startswith("--")]
COLLIDE = "--collide" in sys.argv          # see the docstring: off by default
CHASE = "--chase" in sys.argv              # ride-along camera instead of the
                                           # tracking shot (viewing, not data)
OBSTACLES = "--obstacles" in sys.argv      # synthetic obstacle course on the route
WINDOW = float(args[0]) if len(args) > 0 else 75.0               # half-size, metres
OCC = float(args[1]) if len(args) > 1 else 0.5
NAME = args[2] if len(args) > 2 else "fmi_block"
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
parts.append(f'EXTERNPROTO "{GH}/objects/buildings/protos/SimpleBuilding.proto"')
for path in CAR_PROTOS.values():
    parts.append(f'EXTERNPROTO "{GH}/vehicles/protos/{path}.proto"')
# --chase rides ON the drone (Mounted Shot) instead of trailing it; it only
# changes what the GUI window shows, never the drone's own camera or any
# captured frame. It is a flag rather than a hand edit to the .wbt because a
# hand edit is silently lost the next time the world is regenerated.
VIEWPOINT = ("""  orientation 0 1 0 0.4
  position -0.5 0 0.8
  follow "Mavic 2 PRO"
  followType "Mounted Shot\"""" if CHASE else """  orientation -0.35 0.35 0.87 1.75
  position -8 -14 10
  follow "Mavic 2 PRO"
  followType "Tracking Shot\"""")
# `near` is the other half of the flicker fix (see Z_LAYERS): depth precision is
# governed by the near/far RATIO, so the default 0.05 m near plane against a
# kilometre-deep scene leaves almost no resolution at ground level. Pushing it
# to 0.4 m buys an order of magnitude and costs nothing - the chase camera sits
# 0.5 m from the drone and nothing else is ever that close to the eye.
NEAR = 0.4
parts.append(f"""WorldInfo {{ basicTimeStep 8 }}
Viewpoint {{
{VIEWPOINT}
  near {NEAR}
}}
Background {{ skyColor [ 0.5 0.7 1 ] }}
DirectionalLight {{ direction 0.4 0.5 -1 intensity 2.5 castShadows FALSE }}""")
GROUND = 2 * WINDOW + 200      # ground plane comfortably past the window
# The ground sits BELOW z=0 so the surface stack above it has room to breathe.
# Everything painted on the ground (grass, roads, bay pads, bay lines) is
# stacked within ~10 cm, and at this scene scale (the plane is >1 km across)
# the depth buffer cannot separate surfaces a few millimetres apart - they
# flicker against each other as the viewpoint moves. See Z_LAYERS below.
GROUND_Z = -0.10
parts.append(f"""Solid {{
  translation 0 0 {GROUND_Z}
  name "ground"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor 0.32 0.33 0.34 roughness 1 metalness 0 }}
    geometry Plane {{ size {GROUND:.0f} {GROUND:.0f} }}
  }} ]
  boundingObject Plane {{ size {GROUND:.0f} {GROUND:.0f} }}
}}""")

# Z_LAYERS - the painted-surface stack, bottom to top. The ORDER is a hard
# invariant (roads must cover grass, bay pads must cover roads); the GAPS exist
# only to keep the depth buffer able to tell them apart, and were widened from
# millimetres to centimetres after coplanar flicker showed up in the viewer:
#   ground   -0.100          (Plane)
#   grass     0.005 + 0.004*slot, slot <= 3   -> 105 mm above the ground
#   roads     0.020 + 0.005*slot, slot <= 4 observed   -> 15 mm above grass
#   bay pad   0.070 .. 0.090 -> 30 mm above the highest road
#   bay lines 0.089 .. 0.101 -> 11 mm of line above the pad surface
# Cars (0.4) and the drone (0.15) are well clear of all of it.
ROAD_Z0, ROAD_DZ = 0.020, 0.005
BAY_PAD_Z, BAY_LINE_Z = 0.080, 0.095

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

# --- polygons (scenery: OSM building footprints and green areas). The road
# helpers above are Liang-Barsky on LINES and cannot clip an area, so rings get
# Sutherland-Hodgman; and a concave ring needs explicit triangulation because
# Webots renders non-convex IndexedFaceSet faces unreliably.
# Convention here: a ring is OPEN (no repeated closing point), like the bay
# rings after ring_center()'s [:-1].

def poly_area(ring):
    """Signed shoelace area (positive = counter-clockwise)."""
    n = len(ring)
    return 0.5 * sum(ring[i][0] * ring[(i+1) % n][1] - ring[(i+1) % n][0] * ring[i][1]
                     for i in range(n))

def poly_centroid(ring):
    """Area centroid; falls back to the vertex mean for a degenerate ring."""
    n = len(ring)
    a = poly_area(ring)
    if abs(a) < 1e-9:
        return sum(p[0] for p in ring) / n, sum(p[1] for p in ring) / n
    cx = cy = 0.0
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i+1) % n]
        cr = x0 * y1 - x1 * y0
        cx += (x0 + x1) * cr
        cy += (y0 + y1) * cr
    return cx / (6 * a), cy / (6 * a)

def point_in_poly(p, ring):
    """Ray cast; True if p is inside the ring."""
    x, y = p
    inside = False
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i+1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside

def simplify_ring(ring, tol=0.5):
    """Drop sub-tol steps, then Douglas-Peucker the ring at tol metres."""
    pts = [ring[0]]
    for p in ring[1:]:
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > tol:
            pts.append(p)
    if len(pts) > 2 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) <= tol:
        pts.pop()
    if len(pts) < 4:
        return pts

    def dp(seq):
        if len(seq) < 3:
            return seq
        a, b = seq[0], seq[-1]
        k, dmax = 0, -1.0
        for i in range(1, len(seq) - 1):
            d = _pt_seg_d2(seq[i][0], seq[i][1], a[0], a[1], b[0], b[1])
            if d > dmax:
                k, dmax = i, d
        if dmax <= tol * tol:
            return [a, b]
        return dp(seq[:k+1])[:-1] + dp(seq[k:])

    # DP the closed ring as a path that starts and ends on the first vertex
    out = dp(pts + [pts[0]])[:-1]
    return out if len(out) >= 3 else pts

def clip_polygon(ring, lim):
    """Sutherland-Hodgman: the part of the ring inside |x|,|y| <= lim ([] if none)."""
    def half(poly, keep, cut):
        out = []
        for i in range(len(poly)):
            cur, prv = poly[i], poly[i-1]
            if keep(cur):
                if not keep(prv):
                    out.append(cut(prv, cur))
                out.append(cur)
            elif keep(prv):
                out.append(cut(prv, cur))
        return out

    edges = (
        (lambda p: p[0] <= lim,  lambda a, b: (lim,  a[1] + (b[1]-a[1]) * (lim - a[0]) / (b[0]-a[0]))),
        (lambda p: p[0] >= -lim, lambda a, b: (-lim, a[1] + (b[1]-a[1]) * (-lim - a[0]) / (b[0]-a[0]))),
        (lambda p: p[1] <= lim,  lambda a, b: (a[0] + (b[0]-a[0]) * (lim - a[1]) / (b[1]-a[1]),  lim)),
        (lambda p: p[1] >= -lim, lambda a, b: (a[0] + (b[0]-a[0]) * (-lim - a[1]) / (b[1]-a[1]), -lim)),
    )
    poly = list(ring)
    for keep, cut in edges:
        if not poly:
            return []
        poly = half(poly, keep, cut)
    return poly if len(poly) >= 3 else []

def _in_tri(p, a, b, c):
    d1 = (p[0]-b[0]) * (a[1]-b[1]) - (a[0]-b[0]) * (p[1]-b[1])
    d2 = (p[0]-c[0]) * (b[1]-c[1]) - (b[0]-c[0]) * (p[1]-c[1])
    d3 = (p[0]-a[0]) * (c[1]-a[1]) - (c[0]-a[0]) * (p[1]-a[1])
    return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))

def triangulate(ring):
    """Ear clipping -> list of (i, j, k) index triples, counter-clockwise.
    Returns [] on a self-intersecting ring (no ear found) rather than garbage."""
    if len(ring) < 3:
        return []
    idx = list(range(len(ring)))
    if poly_area(ring) < 0:
        idx.reverse()          # work counter-clockwise
    tris = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k-1], idx[k], idx[(k+1) % len(idx)]
            a, b, c = ring[i0], ring[i1], ring[i2]
            if (b[0]-a[0]) * (c[1]-a[1]) - (b[1]-a[1]) * (c[0]-a[0]) <= 0:
                continue       # reflex vertex, not an ear
            if any(_in_tri(ring[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue       # another vertex sits in the ear
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            return []          # degenerate ring
    tris.append(tuple(idx))
    return tris

# Building heights. OSM gives either building:levels or a metric height; where
# there is neither, Lozenets is panel blocks, so assume DEFAULT_LEVELS.
FLOOR_H = 3.0
DEFAULT_LEVELS = 4
MAX_LEVELS = 40

def _num(s):
    """A number out of a raw OSM tag ('12', '12,5', '12 m', "40'", '5;6')."""
    if not s:
        return None
    s = str(s).split(";")[0].strip().lower().replace(",", ".")
    feet = s.endswith("'") or s.endswith("ft")
    s = s.rstrip("'").removesuffix("ft").removesuffix("m").strip()
    try:
        v = float(s)
    except ValueError:
        return None
    return v * 0.3048 if feet else v

def parse_height(pr):
    """(floor_count, floor_height) for SimpleBuilding, from the raw OSM tags."""
    lv = _num(pr.get("levels"))
    if lv and lv >= 1:
        return min(MAX_LEVELS, int(round(lv))), FLOOR_H
    h = _num(pr.get("height"))
    if h and h > 0:
        n = max(1, min(MAX_LEVELS, int(round(h / FLOOR_H))))
        return n, h / n            # keep the TOTAL height exact
    return DEFAULT_LEVELS, FLOOR_H

def _pt_seg_d2(px, py, ax, ay, bx, by):
    """Squared distance from point to segment."""
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    t = 0.0 if d2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / d2))
    ex, ey = ax + t * dx - px, ay + t * dy - py
    return ex * ex + ey * ey

def _seg_seg_d2(p, q, r, s):
    """Squared distance between segments pq and rs (0 if they intersect)."""
    d1 = (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])
    d2 = (q[0]-p[0])*(s[1]-p[1]) - (q[1]-p[1])*(s[0]-p[0])
    d3 = (s[0]-r[0])*(p[1]-r[1]) - (s[1]-r[1])*(p[0]-r[0])
    d4 = (s[0]-r[0])*(q[1]-r[1]) - (s[1]-r[1])*(q[0]-r[0])
    if d1 * d2 < 0 and d3 * d4 < 0:
        return 0.0
    return min(_pt_seg_d2(*r, *p, *q), _pt_seg_d2(*s, *p, *q),
               _pt_seg_d2(*p, *r, *s), _pt_seg_d2(*q, *r, *s))

def runs_overlap(run_a, wa, run_b, wb):
    """Do two road runs (centreline polylines with widths) overlap on the
    ground? Conservative bbox prefilter, then exact segment distances."""
    gap = (wa + wb) / 2 + 0.3
    ax0 = min(p[0] for p in run_a); ax1 = max(p[0] for p in run_a)
    ay0 = min(p[1] for p in run_a); ay1 = max(p[1] for p in run_a)
    bx0 = min(p[0] for p in run_b); bx1 = max(p[0] for p in run_b)
    by0 = min(p[1] for p in run_b); by1 = max(p[1] for p in run_b)
    if ax0 - gap > bx1 or bx0 - gap > ax1 or ay0 - gap > by1 or by0 - gap > ay1:
        return False
    g2 = gap * gap
    return any(_seg_seg_d2(p, q, r, s) < g2
               for p, q in zip(run_a, run_a[1:])
               for r, s in zip(run_b, run_b[1:]))

road_runs = []   # (osm name, projected polyline) - also feeds the route builder
n_roads = 0
if os.path.exists(ROADS):
    lim = WINDOW + ROAD_MARGIN
    emit = []     # (properties, run) in stable order
    for f in json.load(open(ROADS, encoding="utf-8"))["features"]:
        pts = [((lon - lon0) * mlon, (lat - lat0) * mlat)
               for lon, lat in f["geometry"]["coordinates"]]
        pr = f["properties"]
        for run in clip_polyline(pts, lim):
            road_runs.append((pr.get("name") or "", run))
            emit.append((pr, run))
    # Roads must stay BELOW the bay pads (see Z_LAYERS): a plain per-road
    # stagger grows past the pads once a world has enough runs and paints the
    # road OVER the parking strips — on the 1 km world this covered whole
    # longitudinal bay rows in dark asphalt and produced 330 occupancy false
    # positives. Stagger is only needed between roads that actually overlap
    # (junctions), so assign each run the lowest z-slot not used by any earlier
    # overlapping run (greedy colouring): crossing roads keep their separation,
    # and the max z stays bounded by the junction degree, not the road count.
    slots = []    # slot index per emitted run
    for i, (pr, run) in enumerate(emit):
        used = {slots[j] for j in range(i)
                if runs_overlap(run, road_width(pr), emit[j][1], road_width(emit[j][0]))}
        slot = next(k for k in range(len(used) + 1) if k not in used)
        slots.append(slot)
    pad_bottom = BAY_PAD_Z - 0.01
    if slots and ROAD_Z0 + ROAD_DZ * max(slots) >= pad_bottom:
        print(f"WARNING: road z-slot {max(slots)} reaches the bay-pad layer "
              f"(z >= {pad_bottom:.3f}) - roads may paint over parking bays")
    for (pr, run), slot in zip(emit, slots):
        wpts = ", ".join(f"{x:.2f} {y:.2f} 0" for x, y in run)
        parts.append(f"""Road {{
  translation 0 0 {ROAD_Z0 + ROAD_DZ * slot:.3f}
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

# --- scenery: OSM building footprints and green areas (block_areas.geojson,
# written by tools/get_areas.py), projected with the same lon0/lat0.
#
# Buildings and greens are windowed DIFFERENTLY, on purpose:
#   * a green area is ground paint, so it is CLIPPED - an unclipped 1 km park
#     would carpet the whole visible ground of the 75 m world;
#   * a building is a 3D object, so it is kept or dropped WHOLE by its centroid.
#     The ground plane runs to 2*WINDOW+200 anyway, and a clipped footprint can
#     come back self-touching, which breaks SimpleBuilding's roof triangulation.
# The count caps are inert at WINDOW 75/130 and only bite at 500; largest-area
# first, so what the 1 km world loses is the least visually significant.
BUILDING_MARGIN = 40.0
GREEN_MARGIN = 10.0
MIN_BUILDING_AREA = 25.0      # m2 - drops shed/garage noise
MIN_GREEN_AREA = 40.0
MAX_CORNERS = 24              # past this a building becomes its bounding box
MAX_GREEN_CORNERS = 64        # ear clipping is O(n^3); simplify harder instead
MAX_BUILDINGS = 1400
MAX_GREENS = 300

def obb_ring(ring):
    """The footprint's oriented bounding box (PCA axis), as a 4-corner ring."""
    (cx, cy), (dx, dy) = fit_axis(ring)
    us = [(p[0] - cx) * dx + (p[1] - cy) * dy for p in ring]
    vs = [-(p[0] - cx) * dy + (p[1] - cy) * dx for p in ring]
    mu, mv = (min(us) + max(us)) / 2, (min(vs) + max(vs)) / 2
    return rect_corners(cx + mu * dx - mv * dy, cy + mu * dy + mv * dx,
                        math.atan2(dy, dx), max(us) - min(us), max(vs) - min(vs))

buildings, greens = [], []
if os.path.exists(AREAS):
    drop = {}
    def _drop(why):
        drop[why] = drop.get(why, 0) + 1
    n_boxed = 0
    for f in json.load(open(AREAS, encoding="utf-8"))["features"]:
        pr = f["properties"]
        ring = [((lon - lon0) * mlon, (lat - lat0) * mlat)
                for lon, lat in f["geometry"]["coordinates"][0][:-1]]
        ring = simplify_ring(ring, 0.5)
        if len(ring) < 3:
            _drop("degenerate")
            continue
        if pr.get("kind") == "building":
            cx, cy = poly_centroid(ring)
            if abs(cx) > WINDOW + BUILDING_MARGIN or abs(cy) > WINDOW + BUILDING_MARGIN:
                _drop("outside")
                continue
            area = abs(poly_area(ring))
            if area < MIN_BUILDING_AREA:
                _drop("small")
                continue
            if len(ring) > MAX_CORNERS:
                ring = obb_ring(ring)
                cx, cy = poly_centroid(ring)
                n_boxed += 1
            levels, floor_h = parse_height(pr)
            buildings.append({"id": pr.get("osm_id"), "ring": ring, "cx": cx, "cy": cy,
                              "levels": levels, "floor_h": floor_h, "area": area,
                              "top": levels * floor_h})
        else:
            ring = clip_polygon(ring, WINDOW + GREEN_MARGIN)
            if len(ring) < 3:
                _drop("outside")
                continue
            for tol in (0.5, 1.0, 2.0, 4.0, 8.0):
                if len(ring) <= MAX_GREEN_CORNERS:
                    break
                ring = simplify_ring(ring, tol)
            if len(ring) > MAX_GREEN_CORNERS:
                ring = obb_ring(ring)
                n_boxed += 1
            area = abs(poly_area(ring))
            if area < MIN_GREEN_AREA:
                _drop("small")
                continue
            kind = pr.get("landuse") or pr.get("leisure") or pr.get("natural") or "grass"
            greens.append({"id": pr.get("osm_id"), "ring": ring, "kind": kind, "area": area})

    buildings.sort(key=lambda b: -b["area"])
    greens.sort(key=lambda g: -g["area"])
    if len(buildings) > MAX_BUILDINGS:
        drop["over cap"] = drop.get("over cap", 0) + len(buildings) - MAX_BUILDINGS
        del buildings[MAX_BUILDINGS:]
    if len(greens) > MAX_GREENS:
        drop["over cap"] = drop.get("over cap", 0) + len(greens) - MAX_GREENS
        del greens[MAX_GREENS:]
    print(f"scenery: {len(buildings)} buildings, {len(greens)} green areas"
          f"   (dropped {dict(sorted(drop.items()))}, {n_boxed} reduced to a bounding box)")
else:
    print("WARNING: no block_areas.geojson (run tools/get_areas.py) - world has no scenery")

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
    out = [solid(b["x"], b["y"], BAY_PAD_Z, b["ang"], b["L"], b["W"], 0.02,
                 BAY_ASPHALT, name=f"bay_{b['id']}")]
    # (u, v) = centre of each line in the bay frame (u along length, v across),
    # with the line's box size; lines sit just above the pad
    lines = [(0, +(b["W"] - LINE_W) / 2, b["L"], LINE_W),
             (0, -(b["W"] - LINE_W) / 2, b["L"], LINE_W),
             (+(b["L"] - LINE_W) / 2, 0, LINE_W, b["W"]),
             (-(b["L"] - LINE_W) / 2, 0, LINE_W, b["W"])]
    for i, (u, v, sx, sy) in enumerate(lines):
        out.append(solid(b["x"] + u * ca - v * sa, b["y"] + u * sa + v * ca,
                         BAY_LINE_Z, b["ang"], sx, sy, 0.012,
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

# --- forward obstacle sensors, mounted in the Mavic2Pro's bodySlot.
#
# Emitted ONLY with --collide, and that is deliberate rather than lazy: a range
# sensor in a world whose scenery has no bounding object reads max range
# forever, so the rig would be pure cost (9 ray casts per 8 ms step) buying
# nothing. --collide already means "this is an obstacle world", so it arms the
# whole chain - buildings, poles, and now the drone's own sensing.
#
# A FAN OF SINGLE-RAY SENSORS, not a Lidar, because it is what the hardware plan
# actually specifies (Pixhawk + RPi + range finders) and because its weakness is
# the thing worth measuring: rays diverge, so a thin obstacle fits between two of
# them at range. That is exactly what the course's 0.9 m `mast` is for. If the
# mast turns out to be undetectable in time, THAT is the evidence for swapping in
# a Lidar - not an assumption made up front.
#
# Range is 80 m and not something tidier because braking is the binding
# constraint: A_BRAKE is ~0.22 m/s^2 (TILT_MAX caps the tilt, and there is no
# drag in the sim), so stopping from V_MAX 5 m/s takes v^2/2a = 57 m. A 40 m
# sensor could not stop this drone at cruise speed no matter how good the logic.
# DS_N / DS_SPREAD_DEG / DS_RANGE are duplicated in controllers/parkdrone —
# keep them in step, same convention as ORIGIN.
DS_N = 9                # rays in the fan (odd, so one looks straight ahead)
DS_SPREAD_DEG = 40.0    # half-width of the fan
DS_RANGE = 80.0         # m; must exceed the 57 m brake distance from V_MAX
DS_X = 0.35             # mounted ahead of the body so the props aren't in view

sensor_slot = ""
if COLLIDE:
    ds = []
    for i in range(DS_N):
        a = math.radians(-DS_SPREAD_DEG + 2 * DS_SPREAD_DEG * i / (DS_N - 1))
        # a DistanceSensor casts along its own +x; rotating about z by +a swings
        # that ray to the left. type "laser" is the same single ray as "generic"
        # but Webots DRAWS it, which is the whole debugging story on the chase cam.
        ds.append(f"""    DistanceSensor {{
      translation {DS_X} 0 0
      rotation 0 0 1 {a:.4f}
      name "ds_{i}"
      type "laser"
      resolution -1
      lookupTable [ 0 0 0, {DS_RANGE:.1f} {DS_RANGE:.1f} 0 ]
    }}""")
    sensor_slot = "\n  bodySlot [\n" + "\n".join(ds) + "\n  ]"

# drone at centre (Mavic2Pro ships a gimbal camera named "camera");
# controllerArgs tells parkdrone which route file belongs to THIS world
parts.append(f"""Mavic2Pro {{
  translation 0 0 0.15
  controller "parkdrone"
  controllerArgs [ "{ROUTE_FILE}" ]{sensor_slot}
}}""")
if COLLIDE:
    print(f"drone: {DS_N}-ray forward sensor fan, +/-{DS_SPREAD_DEG:.0f} deg, "
          f"{DS_RANGE:.0f} m range")

# ---------------------------------------------------------------------------
# Scenery emission. Appended AFTER the drone so the road/bay/car region of the
# generated .wbt stays byte-identical - a 2 MB generated file is only
# reviewable if new content lands at the end.
rng_scene = random.Random(11)   # third stream: cosmetics only, so that neither
                                # ground_truth.json (module random, seed 42) nor
                                # the parked cars (rng_cars, seed 7) can shift

# Green areas are painted at z = 0.005: above the ground plane (0.0) and BELOW
# the roads (0.01+), so a park polygon that crosses a street renders under the
# asphalt and can never cover a bay pad (0.04-0.06) or its paint (0.059-0.071).
# Per the "harden the scene" decision, greenery is NOT subtracted around bays -
# it runs to the kerb and under bay rows wherever OSM says so.
# Greens also have to be separated from EACH OTHER: OSM stacks a lawn inside a
# park inside a wood, and coplanar overlapping polygons dither against one
# another exactly like the roads did at junctions. Same greedy slot colouring,
# with a bbox overlap test (conservative - a false "overlap" only costs a slot).
# Sorted largest-area first, so the smaller polygon lands on top, which is also
# the right visual order.
GREEN_Z0, GREEN_DZ = 0.005, 0.004      # slot 0..3 stays clear of ROAD_Z0

def _bbox(ring):
    xs = [p[0] for p in ring]; ys = [p[1] for p in ring]
    return min(xs), min(ys), max(xs), max(ys)

green_slots = []
green_boxes = []
for g in greens:
    box = _bbox(g["ring"])
    used = {green_slots[j] for j in range(len(green_boxes))
            if not (box[2] < green_boxes[j][0] or box[0] > green_boxes[j][2] or
                    box[3] < green_boxes[j][1] or box[1] > green_boxes[j][3])}
    green_slots.append(next(k for k in range(len(used) + 1) if k not in used))
    green_boxes.append(box)
if green_slots and GREEN_Z0 + GREEN_DZ * max(green_slots) >= ROAD_Z0:
    print(f"WARNING: green z-slot {max(green_slots)} reaches the road layer "
          f"(z >= {ROAD_Z0}) - grass may paint over streets")

GREEN_COLORS = {"grass": (0.35, 0.52, 0.24), "meadow": (0.35, 0.52, 0.24),
                "park": (0.32, 0.48, 0.22), "garden": (0.32, 0.48, 0.22),
                "forest": (0.20, 0.35, 0.16), "wood": (0.20, 0.35, 0.16),
                "scrub": (0.24, 0.38, 0.18)}

n_green = 0
for g, slot in zip(greens, green_slots):
    gz = GREEN_Z0 + GREEN_DZ * slot
    tris = triangulate(g["ring"])
    if not tris:
        continue                # self-intersecting ring; skip rather than emit garbage
    r, gr, b = GREEN_COLORS.get(g["kind"], GREEN_COLORS["grass"])
    jit = lambda c: max(0.0, min(1.0, c + rng_scene.uniform(-0.02, 0.02)))
    pts = ", ".join(f"{x:.2f} {y:.2f} {gz:.3f}" for x, y in g["ring"])
    idx = " ".join(f"{i} {j} {k} -1" for i, j, k in tris)
    parts.append(f"""Solid {{
  name "green_{g['id']}"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {jit(r):.3f} {jit(gr):.3f} {jit(b):.3f} roughness 1 metalness 0 }}
    geometry IndexedFaceSet {{
      coord Coordinate {{ point [ {pts} ] }}
      coordIndex [ {idx} ]
    }}
  }} ]
}}""")
    n_green += 1

# Diagnostic, not a filter: how many bays now sit on grass. If the classifier's
# accuracy moves after this change, this number is the first thing to look at.
on_grass = sum(1 for b in bays
               if any(point_in_poly((b["x"], b["y"]), g["ring"]) for g in greens))
if greens:
    print(f"scenery: {n_green} green areas painted   ({on_grass} bays sit on one)")

# Buildings: SimpleBuilding takes the footprint ring directly (`corners`, node-
# local), fixes the winding itself and handles non-convex rings, so no hand-
# written IndexedFaceSet. Flat roofs - Sofia panel blocks, and it also avoids
# the proto degrading a "pyramidal roof" on every real L-shaped footprint.
WALL_TYPES = ["residential building", "old building", "windowed building",
              "concrete building", "brick building"]   # no glass towers: Lozenets
ROOF_TYPES = ["bitumen", "gravel", "sheet metal"]   # what a nadir camera sees

for bl in buildings:
    corners = ", ".join(f"{x - bl['cx']:.2f} {y - bl['cy']:.2f}" for x, y in bl["ring"])
    parts.append(f"""SimpleBuilding {{
  translation {bl['cx']:.3f} {bl['cy']:.3f} 0
  name "bld_{bl['id']}"
  corners [ {corners} ]
  floorHeight {bl['floor_h']:.2f}
  floorNumber {bl['levels']}
  wallType "{rng_scene.choice(WALL_TYPES)}"
  roofType "{rng_scene.choice(ROOF_TYPES)}"
  roofShape "flat roof"
  enableBoundingObject {"TRUE" if COLLIDE else "FALSE"}
}}""")

if buildings:
    tall = max(bl["top"] for bl in buildings)
    print(f"scenery: {len(buildings)} buildings   (tallest {tall:.0f} m, "
          f"bounding objects {'ON' if COLLIDE else 'off'})")

# Light poles, placed PROCEDURALLY along the street centerlines rather than from
# OSM highway=street_lamp: those nodes are barely mapped in Sofia, so most of the
# block would get none, and the density would vary with the window for no reason
# we control. Walking the centerlines puts a pole exactly where it matters - at
# the kerb, beside a bay row. The spacing opens up on bigger worlds so the count
# stays bounded (25 m at half-size 75/130, ~50 m at 500).
MAX_POLES = 400
POLE_SPACING_MIN = 25.0
POLE_KERB = 1.0        # metres out from the road edge
POLE_MIN_SEP = 3.0     # dedup at junctions, where runs meet

poles = []
if os.path.exists(ROADS) and emit:
    lamp_runs = [(road_width(pr), list(run)) for pr, run in emit]   # copy: build_route
    total_len = sum(math.hypot(b[0]-a[0], b[1]-a[1])                # reads road_runs
                    for _, run in lamp_runs for a, b in zip(run, run[1:]))
    spacing = max(POLE_SPACING_MIN, total_len / MAX_POLES)
    side = 1
    for width, run in lamp_runs:
        s = spacing / 2      # don't start every street with a pole on its corner
        for a, b in zip(run, run[1:]):
            seg = math.hypot(b[0]-a[0], b[1]-a[1])
            if seg < 1e-6:
                continue
            ux, uy = (b[0]-a[0]) / seg, (b[1]-a[1]) / seg
            while s < seg:
                at = s
                s += spacing
                off = width / 2 + POLE_KERB
                # try the alternating side first, then the other one: a bay row
                # occupies one kerb, and rejecting outright would leave whole
                # streets unlit
                for sd in (side, -side):
                    px = a[0] + ux * at - uy * off * sd
                    py = a[1] + uy * at + ux * off * sd
                    if abs(px) > WINDOW or abs(py) > WINDOW:
                        continue
                    # a pole standing on painted tarmac is a ground-truth bug,
                    # not scene hardening: that bay could never be occupied.
                    # The test is the bay RECTANGLE plus a margin, not its
                    # circumscribed circle - a lamp between two bays is exactly
                    # where a real one stands.
                    if any(abs((px-bb["x"])*math.cos(bb["ang"]) + (py-bb["y"])*math.sin(bb["ang"])) < bb["L"]/2 + 0.4
                           and abs(-(px-bb["x"])*math.sin(bb["ang"]) + (py-bb["y"])*math.cos(bb["ang"])) < bb["W"]/2 + 0.4
                           for bb in bays):
                        continue
                    if any((px-qx)**2 + (py-qy)**2 < POLE_MIN_SEP**2 for qx, qy, _, _ in poles):
                        continue
                    poles.append((px, py, math.atan2(uy, ux), sd))
                    break
                side = -side
            s -= seg

# A lamp is a mast, an arm and a head - built here rather than taken from the
# StreetLight proto. The proto is a 66 KB detailed mesh AND carries a live
# SpotLight (intensity 30, radius 1000); 375 of them on the 1 km world slowed
# capture from ~18 s to ~90 s per waypoint (a 49 h patrol) and would have
# shifted the daylight exposure the classifier's brightness thresholds are
# calibrated against. From 30 m a pole covers about 3 px, so the detail bought
# nothing. This is a daytime scene: lamps are geometry, not light sources.
# NB: in the R2023b ENU coordinate system a Cylinder's axis is already Z, so the
# mast needs NO rotation - adding the VRML-era `rotation 1 0 0 pi/2` lays it flat.
POLE_H, POLE_R = 7.0, 0.09
ARM_L, HEAD_L = 1.6, 0.5
POLE_COLOR = "0.25 0.25 0.27"
# A pole is 7 m tall and the drone cruises at 30, so this collision geometry can
# never be hit in level flight - it exists so a range sensor can SEE the mast
# (Webots range devices only return nodes that have a bounding object), which
# matters the moment avoidance is allowed to descend or the pole stands next to
# a building the drone is skirting. Mast only: the arm and head are 10 cm boxes
# 7 m up, not worth the extra collision shape.
POLE_BOUND = f"""
  boundingObject Pose {{ translation 0 0 {POLE_H/2:.2f}
    children [ Cylinder {{ height {POLE_H} radius {POLE_R} }} ] }}""" if COLLIDE else ""

for i, (px, py, ang, sd) in enumerate(poles):
    arm = -sd * (ARM_L / 2)      # the arm reaches OUT over the road
    head = -sd * (ARM_L + HEAD_L / 2 - 0.1)
    parts.append(f"""Solid {{
  translation {px:.2f} {py:.2f} 0
  rotation 0 0 1 {ang:.4f}
  name "lamp_{i}"
  children [
    Pose {{ translation 0 0 {POLE_H/2:.2f} children [ Shape {{
      appearance PBRAppearance {{ baseColor {POLE_COLOR} roughness 0.6 metalness 0.4 }}
      geometry Cylinder {{ height {POLE_H} radius {POLE_R} }} }} ] }}
    Pose {{ translation 0 {arm:.2f} {POLE_H - 0.05:.2f} children [ Shape {{
      appearance PBRAppearance {{ baseColor {POLE_COLOR} roughness 0.6 metalness 0.4 }}
      geometry Box {{ size 0.10 {ARM_L} 0.10 }} }} ] }}
    Pose {{ translation 0 {head:.2f} {POLE_H - 0.12:.2f} children [ Shape {{
      appearance PBRAppearance {{ baseColor 0.30 0.30 0.32 roughness 0.5 metalness 0.4 }}
      geometry Box {{ size 0.28 {HEAD_L} 0.14 }} }} ] }}
  ]{POLE_BOUND}
}}""")
if poles:
    print(f"scenery: {len(poles)} light poles   (every {spacing:.0f} m of street)")

route = build_route(bays, road_runs)

# ---------------------------------------------------------------------------
# --- synthetic obstacle course (--obstacles). Four deliberate conflicts, each
# anchored to a FRACTION along the finished route so the course scales with
# whatever window it is generated at, and each tall enough to reach the 30 m
# cruise level. They are emitted here, after build_route, because they are
# placed relative to the route - and they are NEVER fed back into it: the route
# stays the survey route, and running into these is the whole point.
#
# The four are different failure modes, not four copies of one test:
#   tower  head-on, wide - the easy case, must be seen and gone around;
#   slab   offset to one side, long - clips the corridor without blocking it,
#          so an avoider that only reacts to a centred return will graze it;
#   mast   0.9 m across - the sparse-DistanceSensor-fan blind spot: a narrow
#          obstacle can sit between two rays at range and appear only when it
#          is too late to brake (this is the case that argues for a Lidar);
#   trap   a U opening TOWARD the drone - the classic concave deadlock. Pure
#          reactive avoidance circles inside it forever, which is why the
#          design calls for a timeout-and-escape hatch. Expect this one to
#          fail first; that is what it is for.
#
# 'along' is the extent in the direction of travel, 'across' is lateral, 'off'
# shifts the whole structure left(+)/right(-) of the route centreline.
OBSTACLE_COURSE = [
    {"name": "tower", "at": 0.10, "off":  0.0, "shape": "box",  "along":  8.0, "across":  8.0, "h": 45.0},
    {"name": "slab",  "at": 0.22, "off":  7.0, "shape": "box",  "along": 18.0, "across":  6.0, "h": 38.0},
    {"name": "mast",  "at": 0.35, "off":  0.0, "shape": "mast", "along":  0.9, "across":  0.9, "h": 40.0},
    {"name": "trap",  "at": 0.50, "off":  0.0, "shape": "u",    "along": 14.0, "across": 20.0, "h": 34.0},
]
OBST_WALL = 2.0          # wall thickness of the U trap (m)
OBST_NUDGE = 12          # waypoints to search past the anchor for a clear spot
OBST_MIN_SEP = 30.0      # metres between structures. Route fractions are not
                         # enough on their own: a postman route doubles back, so
                         # two anchors 12 waypoints apart can land 15 m apart on
                         # the same street and merge into one encounter
OBST_CONCRETE = "0.58 0.56 0.52"
OBST_MAST = "0.85 0.45 0.10"    # hazard orange: the thin one has to be findable

def obstacle_boxes(spec, px, py, ang):
    """(cx, cy, along, across, name_suffix) boxes making up one structure,
    placed at (px, py) with the route heading ang."""
    ca, sa = math.cos(ang), math.sin(ang)
    def at(u, v):        # bay-frame (along, across) -> world
        return px + u * ca - v * sa, py + u * sa + v * ca
    off = spec["off"]
    if spec["shape"] == "u":
        d, w, t = spec["along"], spec["across"], OBST_WALL
        # back wall across the path, two side walls running BACK toward the
        # drone, so the mouth faces the approach
        bx, by = at(d, off)
        lx, ly = at(d / 2 - t / 2, off + (w - t) / 2)
        rx, ry = at(d / 2 - t / 2, off - (w - t) / 2)
        return [(bx, by, t, w, "back"), (lx, ly, d - t, t, "left"), (rx, ry, d - t, t, "right")]
    cx, cy = at(0.0, off)
    return [(cx, cy, spec["along"], spec["across"], "")]

obstacle_hazards = []
if OBSTACLES:
    if len(route) < 4:
        print("WARNING: --obstacles needs a route with at least 4 waypoints - skipped")
    else:
        bay_rects = [rect_corners(b["x"], b["y"], b["ang"], b["L"], b["W"]) for b in bays]
        taken, centres = [], []
        for spec in OBSTACLE_COURSE:
            anchor = min(len(route) - 2, max(0, int(spec["at"] * (len(route) - 1))))
            chosen = None
            # walk FORWARD from the anchor for a placement that covers no bay
            # and no earlier structure: a box standing on painted tarmac is a
            # ground-truth bug (that bay could never hold a car), and two
            # structures merged into one is a weaker test than two separate ones
            for step in range(OBST_NUDGE + 1):
                i = anchor + step
                if i >= len(route) - 1:
                    break
                px, py = route[i]
                if any(math.hypot(px - qx, py - qy) < OBST_MIN_SEP for qx, qy in centres):
                    continue
                ang = math.atan2(route[i+1][1] - py, route[i+1][0] - px)
                boxes = obstacle_boxes(spec, px, py, ang)
                rects = [rect_corners(bx, by, ang, bl + 1.0, bw + 1.0)
                         for bx, by, bl, bw, _ in boxes]
                if any(rects_overlap(r, q) for r in rects for q in bay_rects + taken):
                    continue
                chosen = (i, px, py, ang, boxes, rects)
                break
            if chosen is None:
                print(f"WARNING: obstacle '{spec['name']}' found no bay-free spot "
                      f"within {OBST_NUDGE} waypoints of wp{anchor} - not placed")
                continue
            i, px, py, ang, boxes, rects = chosen
            taken.extend(rects)
            centres.append((px, py))
            color = OBST_MAST if spec["shape"] == "mast" else OBST_CONCRETE
            for bx, by, bl, bw, suffix in boxes:
                nm = f"obst_{spec['name']}" + (f"_{suffix}" if suffix else "")
                parts.append(f"""Solid {{
  translation {bx:.3f} {by:.3f} {spec['h']/2:.3f}
  rotation 0 0 1 {ang:.4f}
  name "{nm}"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
    geometry Box {{ size {bl:.3f} {bw:.3f} {spec['h']:.3f} }}
  }} ]
  boundingObject Box {{ size {bl:.3f} {bw:.3f} {spec['h']:.3f} }}
}}""")
            # one hazard entry per structure, ring = the union of its boxes, so
            # the U trap reads as a single obstacle to whatever consumes the file
            obstacle_hazards.append({"id": f"obst_{spec['name']}", "cx": px, "cy": py,
                                     "top": spec["h"], "synthetic": True,
                                     "rings": [rect_corners(bx, by, ang, bl, bw)
                                               for bx, by, bl, bw, _ in boxes]})
            print(f"obstacle: {spec['name']:5s} {spec['h']:.0f} m at wp{i} "
                  f"({px:.1f}, {py:.1f})" + (f", {spec['off']:+.0f} m off-centre"
                                             if spec["off"] else " ON the route"))
        if not COLLIDE and obstacle_hazards:
            print("WARNING: --obstacles without --collide - the buildings and poles "
                  "have no collision geometry, so only the course itself is sensable")

# --- route hazards. The controller holds a fixed altitude and has no obstacle
# logic at all, so a building that reaches the flight level under the route is
# a real conflict - reported whether or not --collide is on, because without
# bounding objects the drone flies THROUGH it and the frames beneath it are
# garbage either way. <NAME>.hazards.json is the handoff to the
# obstacle-avoidance work: it needs exactly this list, in these metres. The
# --obstacles course lands in the same file, flagged "synthetic": true, so a
# controller that arms its sensors off this list needs no special case for the
# test world.
FLIGHT_ALT = 30.0      # keep in step with TARGET_ALT in controllers/parkdrone
VERT_CLEAR = 5.0       # how close to the flight level still counts
LATERAL_MARGIN = 10.0  # how far from the route centerline still counts

# one item per structure; a structure can be several rings (the U trap's walls)
haz_items = [{"id": bl["id"], "rings": [bl["ring"]], "cx": bl["cx"], "cy": bl["cy"],
              "top": bl["top"], "synthetic": False} for bl in buildings]
haz_items += obstacle_hazards

def route_dist(ring):
    """Closest approach of the route to one ring, or None if never in range."""
    rx = [p[0] for p in ring]; ry = [p[1] for p in ring]
    lo_x, hi_x = min(rx) - LATERAL_MARGIN, max(rx) + LATERAL_MARGIN
    lo_y, hi_y = min(ry) - LATERAL_MARGIN, max(ry) + LATERAL_MARGIN
    best = None
    for a, b in zip(route, route[1:]):
        if max(a[0], b[0]) < lo_x or min(a[0], b[0]) > hi_x:
            continue
        if max(a[1], b[1]) < lo_y or min(a[1], b[1]) > hi_y:
            continue
        if point_in_poly(a, ring) or point_in_poly(b, ring):
            return 0.0
        d2 = min(_seg_seg_d2(a, b, ring[i], ring[(i+1) % len(ring)]) for i in range(len(ring)))
        best = d2 if best is None else min(best, d2)
    return None if best is None else math.sqrt(best)

hazards = []
for it in haz_items:
    if it["top"] + VERT_CLEAR <= FLIGHT_ALT:
        continue
    ds = [d for d in (route_dist(r) for r in it["rings"]) if d is not None]
    if not ds:
        continue
    d = min(ds)
    if d <= LATERAL_MARGIN:
        h = {"osm_id": it["id"], "height_m": round(it["top"], 1),
             "min_route_dist_m": round(d, 1),
             "centroid": [round(it["cx"], 2), round(it["cy"], 2)]}
        if it["synthetic"]:
            h["synthetic"] = True
        hazards.append(h)

hazards.sort(key=lambda h: -h["height_m"])
haz_file = os.path.join(WORLDS, f"{NAME}.hazards.json")
json.dump(hazards, open(haz_file, "w"), indent=0)
n_tall = sum(1 for it in haz_items if it["top"] + VERT_CLEAR > FLIGHT_ALT)
if hazards:
    h0 = hazards[0]
    label = h0["osm_id"] if h0.get("synthetic") else f"bld_{h0['osm_id']}"
    print(f"WARNING: {n_tall} structures reach the {FLIGHT_ALT:.0f} m flight level; "
          f"{len(hazards)} lie within {LATERAL_MARGIN:.0f} m of the route "
          f"(tallest: {label} at {h0['height_m']:.0f} m, "
          f"{h0['min_route_dist_m']:.0f} m away)")
elif n_tall:
    print(f"{n_tall} structures reach the {FLIGHT_ALT:.0f} m flight level, none near the route")

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
