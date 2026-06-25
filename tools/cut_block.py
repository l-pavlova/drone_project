#!/usr/bin/env python3
"""
Cut a square demo block around a center point from the Sofia parking spaces.

    python cut_block.py "бул. Джеймс Баучер 5, София" 200   # geocode + 400x400 m block
    python cut_block.py 42.6699,23.3326 200                  # explicit lat,lon

Writes block_spaces.geojson, block_map.html, block_preview.png in ../data.
UTF-8 / urllib only (Git Bash mangles Cyrillic).
"""
import json, sys, ssl, math, os, urllib.request, urllib.parse, collections
sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(__file__), "..", "data")
SPACES = os.path.join(DATA, "spaces_25.geojson")
UA = {"User-Agent": "parkdrone-thesis/1.0 (lusito.pav@gmail.com)"}
CTX = ssl.create_default_context(); CTX.check_hostname = False; CTX.verify_mode = ssl.CERT_NONE

query = sys.argv[1] if len(sys.argv) > 1 else "бул. Джеймс Баучер 5, София"
half_m = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0


def geocode(q):
    full = "https://nominatim.openstreetmap.org/search?" + urllib.parse.urlencode(
        {"q": q, "format": "jsonv2", "limit": "1", "accept-language": "bg", "countrycodes": "bg"})
    req = urllib.request.Request(full, headers=UA)
    with urllib.request.urlopen(req, timeout=60, context=CTX) as r:
        res = json.loads(r.read())
    if not res:
        sys.exit(f"Could not geocode: {q}")
    e = res[0]
    print(f"Geocoded '{q}' -> {e['lat']},{e['lon']}  | {e.get('display_name','')[:70]}")
    return float(e["lat"]), float(e["lon"])


if "," in query and all(p.strip().replace(".", "").replace("-", "").isdigit()
                        for p in query.split(",")):
    lat, lon = map(float, query.split(",")); print(f"Center: {lat},{lon}")
else:
    lat, lon = geocode(query)

dlat = half_m / 111320.0
dlon = half_m / (111320.0 * math.cos(math.radians(lat)))
latmin, latmax, lonmin, lonmax = lat - dlat, lat + dlat, lon - dlon, lon + dlon

spaces = json.load(open(SPACES, encoding="utf-8"))["features"]
kept = []
for f in spaces:
    g = f.get("geometry")
    if not g:
        continue
    x, y = g["coordinates"][0]
    if lonmin <= x <= lonmax and latmin <= y <= latmax:
        kept.append(f)

print(f"\nBlock {2*half_m:.0f}x{2*half_m:.0f} m around ({lat:.5f},{lon:.5f})")
print(f"Spaces in block: {len(kept)}")
print("  by zone:", dict(collections.Counter(f['properties'].get('zona') for f in kept)))
print("  by type:", dict(collections.Counter(f['properties'].get('vid_txt_20') for f in kept).most_common(6)))
print("  by orientation:", dict(collections.Counter(f['properties'].get('park_txt') for f in kept)))
streets = collections.Counter(f['properties'].get('mestopoloz') for f in kept)
print("  streets:", dict(streets.most_common(10)))

json.dump({"type": "FeatureCollection", "features": kept,
           "_center": [lat, lon], "_half_m": half_m},
          open(os.path.join(DATA, "block_spaces.geojson"), "w", encoding="utf-8"), ensure_ascii=False)

# preview png
try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    col = {"Зелена зона": "#1faa3f", "Синя зона": "#1f6fe0", "Извън зона": "#888"}
    fig, ax = plt.subplots(figsize=(8, 8))
    by = collections.defaultdict(lambda: ([], []))
    for f in kept:
        x, y = f["geometry"]["coordinates"][0]; z = f["properties"].get("zona")
        by[z][0].append(x); by[z][1].append(y)
    for z, (xs, ys) in by.items():
        ax.scatter(xs, ys, s=14, c=col.get(z, "#888"), label=f"{z} ({len(xs)})")
    ax.scatter([lon], [lat], marker="*", s=300, c="red", zorder=5, label="ФМИ")
    ax.set_title(f"Demo block — {2*half_m:.0f}m around ФМИ ({len(kept)} spaces)")
    ax.set_aspect(1/math.cos(math.radians(lat))); ax.legend(fontsize=8)
    plt.tight_layout(); plt.savefig(os.path.join(DATA, "block_preview.png"), dpi=95)
    print("  wrote block_preview.png")
except ImportError:
    print("  (matplotlib missing, skipped png)")

print("  wrote block_spaces.geojson")
