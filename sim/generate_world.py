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
import json, os, math, sys, random

HERE = os.path.dirname(__file__)
BAYS = os.path.join(HERE, "..", "data", "block_bays.geojson")
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
# Flight route: fly ALONGSIDE each street, not a blind square lawnmower.
# Bays carry a street name (mestopoloz); bays on one street form a row roughly
# along a line. For each street we fit that line (PCA principal axis), take the
# row's extent as a segment, densify it into waypoints, then chain the streets
# together (greedy nearest-endpoint from the drone's start) into one path. The
# drone flies over the bay-row centreline so the nadir camera frames the bays.
# Route is written in the SAME local metres as the world so poses.json /
# ground_truth.json all share one coordinate frame.
ROUTE_STEP = 18.0     # spacing between waypoints along a street (m)
ROUTE_EXTEND = 8.0    # overshoot past the end bays so the camera covers them (m)
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

def build_route(bays, start=(0.0, 0.0)):
    """One path that flies alongside every street, chained greedily from start."""
    streets = {}
    for b in bays:
        streets.setdefault(b["street"], []).append((b["x"], b["y"]))
    segs = [street_segment(pts) for name, pts in streets.items()
            if len(pts) >= MIN_BAYS_PER_STREET]
    route, cur, remaining = [], start, list(segs)
    while remaining:
        # pick the street whose nearer endpoint is closest; enter from that end.
        def near_dist(seg):
            a, b = seg
            return min(math.hypot(a[0] - cur[0], a[1] - cur[1]),
                       math.hypot(b[0] - cur[0], b[1] - cur[1]))
        seg = min(remaining, key=near_dist)
        remaining.remove(seg)
        a, b = seg
        if math.hypot(b[0] - cur[0], b[1] - cur[1]) < math.hypot(a[0] - cur[0], a[1] - cur[1]):
            a, b = b, a   # traverse from the end nearer to us
        route.extend(densify(a, b, ROUTE_STEP))
        cur = b
    return route


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

parts = []
parts.append(f"#VRML_SIM {WEBOTS_VER} utf8")
parts.append(f'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/{WEBOTS_VER}/projects/robots/dji/mavic/protos/Mavic2Pro.proto"')
parts.append("""WorldInfo { basicTimeStep 8 }
Viewpoint { orientation -0.30 0.30 0.90 1.7 position 0 -90 90 }
Background { skyColor [ 0.5 0.7 1 ] }
DirectionalLight { direction 0.4 0.5 -1 intensity 2.5 castShadows TRUE }
Solid {
  name "ground"
  children [ Shape {
    appearance PBRAppearance { baseColor 0.32 0.33 0.34 roughness 1 metalness 0 }
    geometry Plane { size 600 600 }
  } ]
  boundingObject Plane { size 600 600 }
}""")

for b in bays:
    # painted bay marking (white, thin)
    parts.append(solid(b["x"], b["y"], 0.02, b["ang"], b["L"], b["W"], 0.02,
                       "0.95 0.95 0.95", name=f"bay_{b['id']}"))
    if b["occupied"]:
        col = CAR_COLORS[b["id"] % len(CAR_COLORS)]
        parts.append(solid(b["x"], b["y"], 0.75, b["ang"], 4.5, 1.8, 1.45, col,
                           name=f"car_{b['id']}", rough="0.4", metal="0.3"))

# drone at centre (Mavic2Pro ships a gimbal camera named "camera")
parts.append("""Mavic2Pro {
  translation 0 0 0.15
  controller "parkdrone"
}""")

route = build_route(bays)

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
