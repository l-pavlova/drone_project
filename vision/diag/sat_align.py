"""Compare Sofiaplan bay geometry against SATELLITE imagery (an outside reference).

    python vision/diag/sat_align.py --out vision/diag/out/sat
    python vision/diag/sat_align.py --street "ул. Бяло море" --zoom 19
    python vision/diag/sat_align.py --dets vision/runs/dji_0075_occupancy.json

`real_align.py` and `paint_ground_control.py` test the bays against OUR OWN drone
frames, which means every conclusion they reach is entangled with our pose chain.
This script tests them against imagery nobody in this project produced.

Basemap is Esri World Imagery (XYZ tiles). Sofia is served to **z19 = 0.22 m/px**
-- z20 returns a "map data not yet available" placeholder, so a 2.2 m bay is ~10 px
across and this is a *placement* check ("tarmac or garden?"), not a sub-metre one.

The basemap has a georeference of its own and it can be wrong, so OSM buildings and
road centerlines are drawn as the control: if the OSM footprints sit on the roofs,
the tiles are aligned and a bay that misses the street is the bay's problem.
"""
import argparse
import json
import math
import os
import sys
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

from PIL import Image, ImageDraw                                 # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "data")
CACHE = os.path.join(HERE, "..", "..", "pics", "sat_tiles")

TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
TS = 256
PLACEHOLDER = 2521          # exact byte size of Esri's "not yet available" tile


# ---------------------------------------------------------------- web mercator
def lonlat_to_px(lon, lat, z):
    n = TS * 2 ** z
    x = (lon + 180.0) / 360.0 * n
    s = math.sin(math.radians(lat))
    y = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n
    return x, y


def enu_to_lonlat(x, y):
    """Inverse of score_occupancy.to_enu."""
    return so.ORIGIN[1] + x / so.MLON, so.ORIGIN[0] + y / so.MLAT


def fetch_tile(z, x, y):
    path = os.path.join(CACHE, str(z), str(x), f"{y}.jpg")
    if os.path.exists(path):
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    req = urllib.request.Request(TILE_URL.format(z=z, x=x, y=y),
                                 headers={"User-Agent": "parkdrone-sat-align"})
    import ssl
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    with urllib.request.urlopen(req, context=ctx, timeout=30) as r:
        blob = r.read()
    if len(blob) == PLACEHOLDER:
        raise RuntimeError(f"no imagery at z{z} (placeholder tile) -- try a lower --zoom")
    with open(path, "wb") as f:
        f.write(blob)
    return path


def mosaic(lon0, lat0, lon1, lat1, z):
    """Stitch the tiles covering the bbox. Returns (image, px_origin_x, px_origin_y)."""
    ax, ay = lonlat_to_px(lon0, lat1, z)          # top-left  (min lon, max lat)
    bx, by = lonlat_to_px(lon1, lat0, z)          # bottom-right
    tx0, ty0 = int(ax // TS), int(ay // TS)
    tx1, ty1 = int(bx // TS), int(by // TS)
    w, h = (tx1 - tx0 + 1) * TS, (ty1 - ty0 + 1) * TS
    n = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
    print(f"basemap: z{z}, {n} tiles, {w}x{h} px, "
          f"{156543.03392 * math.cos(math.radians(lat0)) / 2 ** z:.3f} m/px")
    im = Image.new("RGB", (w, h), (30, 30, 30))
    got = 0
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            try:
                im.paste(Image.open(fetch_tile(z, tx, ty)),
                         ((tx - tx0) * TS, (ty - ty0) * TS))
                got += 1
            except Exception as e:                # a hole is better than a crash
                print(f"  tile {z}/{tx}/{ty}: {e}")
    print(f"  {got}/{n} tiles")
    return im, tx0 * TS, ty0 * TS


# ---------------------------------------------------------------------- layers
def load_rings(path, geom_filter=None):
    out = []
    for ft in json.load(open(path, encoding="utf-8"))["features"]:
        g = ft["geometry"]
        if geom_filter and g["type"] != geom_filter:
            continue
        rings = [g["coordinates"][0]] if g["type"] == "Polygon" else [g["coordinates"]]
        for ring in rings:
            out.append(([(c[0], c[1]) for c in ring], ft["properties"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zoom", type=int, default=19)
    ap.add_argument("--street", help="crop to one street's bays (substring of mestopoloz)")
    ap.add_argument("--near", help="ENU window instead of a street: 'x,y' metres")
    ap.add_argument("--half", type=float, default=80.0, help="half-size of --near window")
    ap.add_argument("--pad", type=float, default=30.0, help="metres of margin")
    ap.add_argument("--dets", help="detect_occupancy --json output, drawn as car centres")
    ap.add_argument("--no-osm", action="store_true")
    ap.add_argument("--bays", default=None, help="bay geojson (default data/block_bays.geojson)")
    ap.add_argument("--bays-alt", help="a SECOND bay file, drawn in magenta -- for comparing "
                                       "the committed geometry against a regenerated one")
    ap.add_argument("--out", default=os.path.join(HERE, "out", "sat_align.png"))
    a = ap.parse_args()

    bays = load_rings(a.bays or os.path.join(DATA, "block_bays.geojson"))
    if a.street:
        bays = [b for b in bays if a.street.lower() in (b[1].get("mestopoloz") or "").lower()]
        if not bays:
            sys.exit(f"no bays matching {a.street!r}")
    print(f"{len(bays)} bays" + (f" on {a.street!r}" if a.street else ""))

    if a.near:
        cx, cy = (float(v) for v in a.near.split(","))
        lon0, lat0 = enu_to_lonlat(cx - a.half, cy - a.half)
        lon1, lat1 = enu_to_lonlat(cx + a.half, cy + a.half)
    else:
        lons = [c[0] for r, _ in bays for c in r]
        lats = [c[1] for r, _ in bays for c in r]
        dlon = a.pad / so.MLON
        dlat = a.pad / so.MLAT
        lon0, lon1 = min(lons) - dlon, max(lons) + dlon
        lat0, lat1 = min(lats) - dlat, max(lats) + dlat

    im, ox, oy = mosaic(lon0, lat0, lon1, lat1, a.zoom)

    def P(lon, lat):
        x, y = lonlat_to_px(lon, lat, a.zoom)
        return x - ox, y - oy

    im = im.convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)

    if not a.no_osm:
        for ring, pr in load_rings(os.path.join(DATA, "block_areas.geojson")):
            if pr.get("kind") != "building":
                continue
            d.line([P(*c) for c in ring] + [P(*ring[0])], fill=(80, 200, 255, 200), width=2)
        for ring, pr in load_rings(os.path.join(DATA, "block_roads.geojson"), "LineString"):
            d.line([P(*c) for c in ring], fill=(255, 255, 0, 110), width=2)

    for ring, pr in bays:
        pts = [P(*c) for c in ring]
        d.line(pts + [pts[0]], fill=(255, 60, 60, 255), width=2)

    if a.bays_alt:
        alt = load_rings(a.bays_alt)
        if a.street:
            alt = [b for b in alt
                   if a.street.lower() in (b[1].get("mestopoloz") or "").lower()]
        for ring, pr in alt:
            pts = [P(*c) for c in ring]
            d.line(pts + [pts[0]], fill=(255, 0, 255, 255), width=2)
        print(f"{len(alt)} bays from --bays-alt (magenta)")

    if a.dets:
        res = json.load(open(a.dets, encoding="utf-8"))
        pts = [(p["x"], p["y"]) for p in res.get("unassigned", [])]
        for p in pts:
            lon, lat = enu_to_lonlat(p[0], p[1])
            u, v = P(lon, lat)
            d.ellipse([u - 4, v - 4, u + 4, v + 4], outline=(255, 160, 0, 255), width=2)
        print(f"{len(pts)} detection points from {a.dets}")

    out = Image.alpha_composite(im, ov).convert("RGB")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    out.save(a.out)
    print(f"wrote {a.out}  ({out.size[0]}x{out.size[1]})")
    print("legend: RED bay outlines | CYAN OSM buildings | YELLOW road centerlines"
          + (" | MAGENTA --bays-alt" if a.bays_alt else "")
          + (" | ORANGE detections" if a.dets else ""))


if __name__ == "__main__":
    main()
