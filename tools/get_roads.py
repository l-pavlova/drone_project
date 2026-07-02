#!/usr/bin/env python3
"""
Fetch OSM street centerlines for the demo block (for drawing roads in the sim).

    python get_roads.py [margin_m]     # bbox = block_spaces.geojson _center/_half_m + margin (default 40)

Reads the block window from ../data/block_spaces.geojson (_center, _half_m),
queries the Overpass API for drivable highways, and writes
../data/block_roads.geojson (LineString per way: name, highway, lanes, oneway).
UTF-8 / urllib only (Git Bash mangles Cyrillic).
"""
import json, sys, ssl, math, os, urllib.request, urllib.parse, collections
sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(__file__), "..", "data")
UA = {"User-Agent": "parkdrone-thesis/1.0 (lusito.pav@gmail.com)"}
CTX = ssl.create_default_context(); CTX.check_hostname = False; CTX.verify_mode = ssl.CERT_NONE
OVERPASS = "https://overpass-api.de/api/interpreter"

margin_m = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0

block = json.load(open(os.path.join(DATA, "block_spaces.geojson"), encoding="utf-8"))
lat, lon = block["_center"]
half_m = block["_half_m"] + margin_m
dlat = half_m / 111320.0
dlon = half_m / (111320.0 * math.cos(math.radians(lat)))
bbox = f"{lat - dlat},{lon - dlon},{lat + dlat},{lon + dlon}"   # S,W,N,E

query = f"""[out:json][timeout:60];
way["highway"]["highway"!~"footway|path|steps|cycleway|pedestrian|service|track|construction"]({bbox});
out geom;"""

print(f"Overpass bbox {2*half_m:.0f}x{2*half_m:.0f} m around ({lat:.5f},{lon:.5f})")
req = urllib.request.Request(OVERPASS, data=urllib.parse.urlencode({"data": query}).encode(),
                             headers=UA)
try:
    with urllib.request.urlopen(req, timeout=90, context=CTX) as r:
        res = json.loads(r.read())
except Exception as e:
    print(f"Overpass failed ({e}), retrying once...")
    with urllib.request.urlopen(req, timeout=90, context=CTX) as r:
        res = json.loads(r.read())

feats = []
for el in res.get("elements", []):
    if el.get("type") != "way" or "geometry" not in el:
        continue
    tags = el.get("tags", {})
    feats.append({
        "type": "Feature",
        "properties": {"osm_id": el["id"],
                       "name": tags.get("name"),
                       "highway": tags.get("highway"),
                       "lanes": tags.get("lanes"),
                       "oneway": tags.get("oneway")},
        "geometry": {"type": "LineString",
                     "coordinates": [[p["lon"], p["lat"]] for p in el["geometry"]]},
    })

names = collections.Counter(f["properties"]["name"] for f in feats if f["properties"]["name"])
kinds = collections.Counter(f["properties"]["highway"] for f in feats)
print(f"ways: {len(feats)}")
print("  by highway:", dict(kinds.most_common()))
print("  streets:", dict(names.most_common(20)))

# which bay streets did we cover? (bays name streets like "ул. Димитър Димов")
bays_path = os.path.join(DATA, "block_bays.geojson")
if os.path.exists(bays_path):
    bay_streets = {f["properties"].get("mestopoloz")
                   for f in json.load(open(bays_path, encoding="utf-8"))["features"]}
    road_names = set(names)
    matched = {s for s in bay_streets if s and any(n and (n in s or s in n) for n in road_names)}
    print(f"  bay streets matched: {len(matched)}/{len(bay_streets)}")
    for s in sorted(bay_streets - matched):
        print(f"    no road match: {s}")

out = os.path.join(DATA, "block_roads.geojson")
json.dump({"type": "FeatureCollection", "features": feats,
           "_center": [lat, lon], "_half_m": half_m},
          open(out, "w", encoding="utf-8"), ensure_ascii=False)
print(f"wrote {out}")
