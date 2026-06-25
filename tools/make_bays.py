#!/usr/bin/env python3
"""
Turn parking-space POINTS into oriented BAY RECTANGLES.

Each space in block_spaces.geojson is a single point. We estimate the local
street bearing from the nearest spaces on the SAME street (PCA on their
positions), then draw a ~2.5x5 m rectangle oriented by `park_txt`:
  - Надлъжн (parallel)      -> car long axis ALONG the street
  - Напречн (perpendicular) -> car long axis ACROSS the street
  - Косо (angled)           -> 45 deg to the street

Outputs in ../data: block_bays.geojson (polygons), block_bays.html (Leaflet over
OSM), block_bays_preview.png (zoomed, to eyeball orientation).
"""
import json, os, math, sys, collections
import numpy as np
sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(__file__), "..", "data")
src = json.load(open(os.path.join(DATA, "block_spaces.geojson"), encoding="utf-8"))
feats = src["features"]

# bay dimensions (metres)
PARALLEL = (5.4, 2.2)       # (length along street, width across)
PERP = (4.8, 2.4)           # (depth across street, width along)
ANGLE_DEG = 45

lats = [f["geometry"]["coordinates"][0][1] for f in feats]
lons = [f["geometry"]["coordinates"][0][0] for f in feats]
lat0 = sum(lats) / len(lats)
mlat = 111320.0
mlon = 111320.0 * math.cos(math.radians(lat0))

def to_m(lon, lat):
    return np.array([lon * mlon, lat * mlat])

def to_ll(x, y):
    return [x / mlon, y / mlat]

P = np.array([to_m(lo, la) for lo, la in zip(lons, lats)])   # metre coords
streets = [f["properties"].get("mestopoloz") for f in feats]
by_street = collections.defaultdict(list)
for i, s in enumerate(streets):
    by_street[s].append(i)

def bearing_for(i):
    """Unit vector along the street at point i, from same-street neighbours."""
    idx = by_street[streets[i]]
    pts = P[idx]
    if len(idx) >= 3:
        d = np.linalg.norm(pts - P[i], axis=1)
        k = min(6, len(idx))
        near = pts[np.argsort(d)[:k]]
        c = near - near.mean(axis=0)
        # principal axis via covariance
        w, v = np.linalg.eigh(c.T @ c)
        u = v[:, np.argmax(w)]
    else:
        # fallback: nearest neighbour anywhere
        d = np.linalg.norm(P - P[i], axis=1); d[i] = 1e9
        j = int(np.argmin(d)); u = P[j] - P[i]
        n = np.linalg.norm(u); u = u / n if n else np.array([1.0, 0.0])
    return u / (np.linalg.norm(u) or 1)

def rect(center_m, u, L, W):
    v = np.array([-u[1], u[0]])           # perpendicular
    a, b = L / 2, W / 2
    corners = [center_m + a*u + b*v, center_m + a*u - b*v,
               center_m - a*u - b*v, center_m - a*u + b*v]
    ring = [to_ll(*c) for c in corners]
    ring.append(ring[0])
    return [ring]

out = []
counts = collections.Counter()
for i, f in enumerate(feats):
    pk = f["properties"].get("park_txt")
    u = bearing_for(i)
    if pk == "Напречн":                    # perpendicular: long axis across street
        u2 = np.array([-u[1], u[0]]); L, W = PERP
    elif pk == "Косо":                     # angled 45
        th = math.radians(ANGLE_DEG)
        u2 = np.array([u[0]*math.cos(th)-u[1]*math.sin(th),
                       u[0]*math.sin(th)+u[1]*math.cos(th)]); L, W = PARALLEL
    else:                                  # Надлъжн / default: parallel
        u2 = u; L, W = PARALLEL
    counts[pk] += 1
    pr = dict(f["properties"]); pr["bearing_deg"] = round(math.degrees(math.atan2(u[1], u[0])), 1)
    out.append({"type": "Feature", "properties": pr,
                "geometry": {"type": "Polygon", "coordinates": rect(P[i], u2, L, W)}})

json.dump({"type": "FeatureCollection", "features": out},
          open(os.path.join(DATA, "block_bays.geojson"), "w", encoding="utf-8"), ensure_ascii=False)
print(f"bays written: {len(out)}  orientations: {dict(counts)}")

# ---- Leaflet map over OSM ----
gj = json.dumps({"type": "FeatureCollection", "features": out}, ensure_ascii=False)
html = """<!DOCTYPE html><html><head><meta charset="utf-8"><title>FMI bays</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body,#map{height:100%;margin:0}</style></head><body><div id="map"></div><script>
var bays=__GJ__; var map=L.map('map');
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:20,attribution:'© OSM'}).addTo(map);
var col={'Надлъжн':'#1faa3f','Напречн':'#1f6fe0','Косо':'#e08a1f'};
var layer=L.geoJSON(bays,{style:function(f){return{color:col[f.properties.park_txt]||'#888',weight:1,fillOpacity:0.5};},
 onEachFeature:function(f,l){l.bindPopup(f.properties.mestopoloz+'<br>'+f.properties.park_txt+' · '+f.properties.zona);}}).addTo(map);
map.fitBounds(layer.getBounds());
</script></body></html>"""
open(os.path.join(DATA, "block_bays.html"), "w", encoding="utf-8").write(html.replace("__GJ__", gj))
print("wrote block_bays.html")

# ---- zoomed PNG to verify orientation ----
try:
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPoly
    cx, cy = P.mean(axis=0); R = 120  # metres window
    fig, ax = plt.subplots(figsize=(9, 9))
    col2 = {"Надлъжн": "#1faa3f", "Напречн": "#1f6fe0", "Косо": "#e08a1f"}
    for i, f in enumerate(out):
        ring = f["geometry"]["coordinates"][0]
        xy = np.array([to_m(lo, la) for lo, la in ring])
        if abs(xy[:,0].mean()-cx) > R or abs(xy[:,1].mean()-cy) > R:
            continue
        ax.add_patch(MPoly(xy, closed=True, fc=col2.get(f["properties"]["park_txt"], "#888"),
                           ec="black", lw=0.4, alpha=0.7))
        ax.plot(P[i,0], P[i,1], '.', c='red', ms=2)
    ax.set_xlim(cx-R, cx+R); ax.set_ylim(cy-R, cy+R); ax.set_aspect("equal")
    ax.set_title("FMI bays (120 m window) — red dots = source points")
    plt.tight_layout(); plt.savefig(os.path.join(DATA, "block_bays_preview.png"), dpi=95)
    print("wrote block_bays_preview.png")
except ImportError:
    print("(no matplotlib, skipped png)")
