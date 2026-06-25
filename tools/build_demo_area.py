#!/usr/bin/env python3
"""
Build a demo-area subset of the Sofia parking data.

Fetches an administrative boundary (default: район Лозенец) from OpenStreetMap
Nominatim, clips the parking-space registry (spaces_25.geojson) to it, writes a
GeoJSON subset, prints a summary, and renders a self-contained Leaflet map.

Run from anywhere:
    python build_demo_area.py                 # defaults to Лозенец
    python build_demo_area.py "Средец"        # another district

Everything is UTF-8 / urllib (no shell), to avoid Cyrillic mangling in Git Bash.
"""
import json, sys, ssl, urllib.request, urllib.parse, os

sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(__file__), "..", "data")
SPACES = os.path.join(DATA, "spaces_25.geojson")
AREA = sys.argv[1] if len(sys.argv) > 1 else "Лозенец"
UA = {"User-Agent": "parkdrone-thesis/1.0 (lusito.pav@gmail.com)"}
CTX = ssl.create_default_context()
CTX.check_hostname = False
CTX.verify_mode = ssl.CERT_NONE  # public read-only GETs; Windows cert revocation is flaky


def get(url, params):
    full = url + "?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(full, headers=UA)
    with urllib.request.urlopen(req, timeout=60, context=CTX) as r:
        return json.loads(r.read())


def fetch_boundary(name):
    res = get("https://nominatim.openstreetmap.org/search", {
        "q": f"{name}, Sofia", "format": "jsonv2", "polygon_geojson": "1",
        "limit": "20", "accept-language": "bg", "countrycodes": "bg", "extratags": "1",
    })
    print("Nominatim candidates:")
    for e in res:
        al = e.get("extratags", {}).get("admin_level")
        print(f"  {e.get('osm_type')}/{e.get('osm_id')} {e.get('category')}/{e.get('type')}"
              f" al={al} rank={e.get('place_rank')} | {e.get('display_name','')[:70]}")
    cands = [e for e in res
             if e.get("category") == "boundary" and e.get("type") == "administrative"
             and e.get("name") == name and e.get("geojson", {}).get("type") in ("Polygon", "MultiPolygon")]
    if not cands:
        sys.exit(f"No administrative boundary named '{name}' found.")
    # prefer the most specific (highest place_rank = smallest admin unit)
    best = max(cands, key=lambda e: e.get("place_rank", 0))
    print(f"\nSelected: R{best['osm_id']} '{best['name']}' "
          f"al={best.get('extratags',{}).get('admin_level')} geom={best['geojson']['type']}")
    return best["geojson"]


def pip(x, y, ring):
    inside = False
    n = len(ring); j = n - 1
    for i in range(n):
        xi, yi = ring[i]; xj, yj = ring[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def in_geom(x, y, geom):
    polys = geom["coordinates"] if geom["type"] == "MultiPolygon" else [geom["coordinates"]]
    for poly in polys:
        if pip(x, y, poly[0]) and not any(pip(x, y, h) for h in poly[1:]):
            return True
    return False


def main():
    boundary = fetch_boundary(AREA)
    spaces = json.load(open(SPACES, encoding="utf-8"))["features"]
    kept = []
    for f in spaces:
        g = f.get("geometry")
        if not g:
            continue
        lon, lat = g["coordinates"][0]  # MultiPoint, single point
        if in_geom(lon, lat, boundary):
            kept.append(f)
    print(f"\nSpaces inside {AREA}: {len(kept)} / {len(spaces)}")

    import collections
    zc = collections.Counter(f["properties"].get("zona") for f in kept)
    vc = collections.Counter(f["properties"].get("vid_txt_20") for f in kept)
    print("  by zone:", dict(zc))
    print("  by type:", dict(vc.most_common(8)))

    out = os.path.join(DATA, "lozenets_spaces.geojson")
    json.dump({"type": "FeatureCollection", "features": kept},
              open(out, "w", encoding="utf-8"), ensure_ascii=False)
    bpath = os.path.join(DATA, "lozenets_boundary.geojson")
    json.dump({"type": "Feature", "geometry": boundary, "properties": {"name": AREA}},
              open(bpath, "w", encoding="utf-8"), ensure_ascii=False)
    print(f"  wrote {out}\n  wrote {bpath}")

    render_map(kept, boundary)


def render_map(kept, boundary):
    color = {"Синя зона": "#1f6fe0", "Зелена зона": "#1faa3f", "Извън зона": "#888888"}
    pts = [{"lat": f["geometry"]["coordinates"][0][1],
            "lon": f["geometry"]["coordinates"][0][0],
            "z": f["properties"].get("zona"),
            "s": f["properties"].get("mestopoloz"),
            "t": f["properties"].get("vid_txt_20")} for f in kept]
    html = """<!DOCTYPE html><html><head><meta charset="utf-8">
<title>PARKDRONE demo area</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html,body,#map{height:100%;margin:0}
.legend{background:#fff;padding:8px 10px;font:13px sans-serif;line-height:20px;border-radius:6px}
.legend i{width:12px;height:12px;display:inline-block;margin-right:6px;border-radius:50%}</style>
</head><body><div id="map"></div><script>
var boundary=__BOUNDARY__, pts=__PTS__, color=__COLOR__;
var map=L.map('map');
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  {maxZoom:19,attribution:'© OpenStreetMap'}).addTo(map);
var bl=L.geoJSON(boundary,{style:{color:'#e0431f',weight:2,fill:false}}).addTo(map);
map.fitBounds(bl.getBounds());
pts.forEach(function(p){
  L.circleMarker([p.lat,p.lon],{radius:3,color:color[p.z]||'#888',
    fillColor:color[p.z]||'#888',fillOpacity:0.8,weight:0})
   .bindPopup((p.s||'')+'<br>'+(p.z||'')+' · '+(p.t||'')).addTo(map);
});
var lg=L.control({position:'bottomright'});
lg.onAdd=function(){var d=L.DomUtil.create('div','legend');
 d.innerHTML='<b>Parking spaces</b><br>'+
 '<i style="background:#1f6fe0"></i>Синя зона (blue)<br>'+
 '<i style="background:#1faa3f"></i>Зелена зона (green)<br>'+
 '<i style="background:#888"></i>Извън зона';return d;};
lg.addTo(map);
</script></body></html>"""
    html = (html.replace("__BOUNDARY__", json.dumps(boundary))
                .replace("__PTS__", json.dumps(pts, ensure_ascii=False))
                .replace("__COLOR__", json.dumps(color, ensure_ascii=False)))
    mpath = os.path.join(DATA, "lozenets_map.html")
    open(mpath, "w", encoding="utf-8").write(html)
    print(f"  wrote {mpath}  ({len(pts)} markers)")


if __name__ == "__main__":
    main()
