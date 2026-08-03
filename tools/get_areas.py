#!/usr/bin/env python3
"""
Fetch OSM building footprints and green areas for the demo block (sim scenery).

    python get_areas.py [margin_m]     # bbox = block_spaces.geojson _center/_half_m + margin (default 60)

Reads the block window from ../data/block_spaces.geojson (_center, _half_m),
queries the Overpass API for buildings and greenery, and writes
../data/block_areas.geojson (Polygon per closed way: kind, tags, height/levels).
UTF-8 / urllib only (Git Bash mangles Cyrillic).

The margin is wider than get_roads.py's 40 m because generate_world.py keeps a
building whole when its CENTROID falls within WINDOW + 40.

Two deliberate omissions, both reported in the summary instead of hidden:
  * multipolygon RELATIONS are skipped. Neither consumer can express holes -
    Webots' SimpleBuilding takes a single `corners` ring - so a relation would
    render as a filled blob, i.e. wrong rather than merely missing. (get_roads.py
    already drops non-ways for the same "no ring assembly here" reason.)
  * unclosed ways are skipped: they are lines, not areas.
Height tags are transcribed RAW; generate_world.py's parse_height() interprets
them, the same split as `lanes` -> road_width().
"""
import json, sys, ssl, math, os, urllib.request, urllib.parse, collections
sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(__file__), "..", "data")
UA = {"User-Agent": "parkdrone-thesis/1.0 (lusito.pav@gmail.com)"}
CTX = ssl.create_default_context(); CTX.check_hostname = False; CTX.verify_mode = ssl.CERT_NONE
OVERPASS = "https://overpass-api.de/api/interpreter"

margin_m = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

block = json.load(open(os.path.join(DATA, "block_spaces.geojson"), encoding="utf-8"))
lat, lon = block["_center"]
half_m = block["_half_m"] + margin_m
dlat = half_m / 111320.0
dlon = half_m / (111320.0 * math.cos(math.radians(lat)))
bbox = f"{lat - dlat},{lon - dlon},{lat + dlat},{lon + dlon}"   # S,W,N,E

query = f"""[out:json][timeout:120];
(
  way["building"]["building"!~"^(no|roof)$"]({bbox});
  way["landuse"~"^(grass|meadow|forest)$"]({bbox});
  way["leisure"~"^(park|garden)$"]({bbox});
  way["natural"~"^(scrub|wood)$"]({bbox});
);
out geom;"""

print(f"Overpass bbox {2*half_m:.0f}x{2*half_m:.0f} m around ({lat:.5f},{lon:.5f})")
req = urllib.request.Request(OVERPASS, data=urllib.parse.urlencode({"data": query}).encode(),
                             headers=UA)
try:
    with urllib.request.urlopen(req, timeout=180, context=CTX) as r:
        res = json.loads(r.read())
except Exception as e:
    print(f"Overpass failed ({e}), retrying once...")
    with urllib.request.urlopen(req, timeout=180, context=CTX) as r:
        res = json.loads(r.read())

feats = []
n_rel = n_open = 0
for el in res.get("elements", []):
    if el.get("type") == "relation":
        n_rel += 1
        continue
    if el.get("type") != "way" or "geometry" not in el:
        continue
    ring = [[p["lon"], p["lat"]] for p in el["geometry"]]
    if len(ring) < 4 or ring[0] != ring[-1]:
        n_open += 1
        continue
    tags = el.get("tags", {})
    feats.append({
        "type": "Feature",
        "properties": {"osm_id": el["id"],
                       "kind": "building" if tags.get("building") else "green",
                       "name": tags.get("name"),
                       "building": tags.get("building"),
                       "landuse": tags.get("landuse"),
                       "leisure": tags.get("leisure"),
                       "natural": tags.get("natural"),
                       "height": tags.get("height"),
                       "levels": tags.get("building:levels"),
                       "min_levels": tags.get("building:min_level")},
        "geometry": {"type": "Polygon", "coordinates": [ring]},   # closing point kept
    })

kinds = collections.Counter(f["properties"]["kind"] for f in feats)
print(f"areas: {len(feats)}   (skipped {n_rel} multipolygon relations, {n_open} unclosed ways)")
print("  by kind:", dict(kinds.most_common()))
for key in ("building", "landuse", "leisure", "natural"):
    vals = collections.Counter(f["properties"][key] for f in feats if f["properties"][key])
    if vals:
        print(f"  by {key}:", dict(vals.most_common(12)))

# height-tag coverage: how many buildings can be given a REAL height, and how
# many will fall back to generate_world.py's DEFAULT_LEVELS
blds = [f["properties"] for f in feats if f["properties"]["kind"] == "building"]
n_lv = sum(1 for p in blds if p["levels"])
n_h = sum(1 for p in blds if p["height"] and not p["levels"])
print(f"  building heights: {n_lv} from building:levels, {n_h} from height, "
      f"{len(blds) - n_lv - n_h} with neither (of {len(blds)})")


def area_m2(ring):
    """Shoelace on a local flat-earth projection about the block centre."""
    mlat, mlon = 111320.0, 111320.0 * math.cos(math.radians(lat))
    pts = [((x - lon) * mlon, (y - lat) * mlat) for x, y in ring[:-1]]
    s = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
            for i in range(len(pts)))
    return abs(s) / 2.0


for kind in ("building", "green"):
    ar = sorted(area_m2(f["geometry"]["coordinates"][0])
                for f in feats if f["properties"]["kind"] == kind)
    if ar:
        print(f"  {kind} area m2: min {ar[0]:.0f}  median {ar[len(ar)//2]:.0f}  max {ar[-1]:.0f}")

out = os.path.join(DATA, "block_areas.geojson")
json.dump({"type": "FeatureCollection", "features": feats,
           "_center": [lat, lon], "_half_m": half_m},
          open(out, "w", encoding="utf-8"), ensure_ascii=False)
print(f"wrote {out}")
