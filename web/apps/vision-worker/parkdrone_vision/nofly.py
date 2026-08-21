"""UAS geographical zones (no-fly / restricted airspace) as GeoJSON.

Static national reference data, not state: the Bulgarian CAA publishes one
ED-269 file per revision (`data/bgr_zones_<ddmmyyyy>/…json`, 881 zones as of the
30-07-2026 edition) and it changes when they publish, not when the drone flies.
So it is read from disk and cached in memory rather than living in Postgres —
there is nothing to join it against and nothing to update transactionally. The
map gets it through the API anyway, because the file sits on the Python side
next to the sim and the browser has no access to it, exactly like
`vision/ground_truth.py`.

Why the drone map cares: a PROHIBITED zone is ground the survey may not
legally cover. The FMI block's own origin is clear, but the 1 km world's route
puts 93 of 1976 waypoints inside the Chinese Embassy zone (0–120 m AGL, so no
cruise altitude avoids it) — which is a coverage limit the product should show
rather than a fact discovered afterwards.

Two shape types have to be reconciled: ED-269 geometry is a `Circle`
(centre + radius in metres) or a `Polygon`, and GeoJSON has no circle. Circles
are emitted as regular polygons so a client can draw one layer with one code
path; `circle_radius_m` is kept in the properties for anyone who wants the true
shape back.
"""
import json
import math
import os
import threading

from .config import NOFLY_FILE

# Segments used to approximate a Circle. 48 keeps the worst radial error under
# 0.2% of the radius (1 - cos(pi/48)), i.e. ~10 m on the largest 5 km zone and
# well under a metre on the urban ones — invisible at any map zoom, against a
# payload that grows linearly with this number.
_CIRCLE_SEGMENTS = 48

_MLAT = 111320.0  # metres per degree of latitude (the project-wide constant)

_lock = threading.Lock()
_cache: list[dict] | None = None


def _circle_ring(lon: float, lat: float, radius_m: float) -> list[list[float]]:
    """Regular polygon approximating a circle of `radius_m` about (lon, lat)."""
    mlon = _MLAT * math.cos(math.radians(lat))
    ring = []
    for i in range(_CIRCLE_SEGMENTS):
        th = 2.0 * math.pi * i / _CIRCLE_SEGMENTS
        ring.append([lon + radius_m * math.cos(th) / mlon,
                     lat + radius_m * math.sin(th) / _MLAT])
    ring.append(list(ring[0]))  # GeoJSON rings are closed
    return ring


def _authority(feature: dict) -> dict | None:
    """The contact a pilot would actually call: prefer whoever authorises."""
    entries = feature.get("zoneAuthority") or []
    chosen = next((a for a in entries if a.get("purpose") == "AUTHORIZATION"),
                  entries[0] if entries else None)
    if not chosen:
        return None
    return {
        "name": chosen.get("name"),
        "email": chosen.get("email"),
        "phone": chosen.get("phone"),
        # ISO-8601 duration, e.g. P07DT00H00M — how far ahead to apply
        "interval_before": chosen.get("intervalBefore") or None,
    }


def _features() -> list[dict]:
    """Every zone geometry as a GeoJSON Feature. Cached; parsed once."""
    global _cache
    with _lock:
        if _cache is not None:
            return _cache
        try:
            with open(NOFLY_FILE, encoding="utf-8") as fh:
                doc = json.load(fh)
        except (OSError, ValueError):
            # A deployment without the CAA file simply has no zones to draw.
            # That is the same "absent labels" posture as ground_truth.py: an
            # empty layer, not a 500.
            _cache = []
            return _cache

        out: list[dict] = []
        for z in doc.get("features", []):
            # One ED-269 zone can carry several volumes with DIFFERENT altitude
            # bands, so each becomes its own Feature — a client drawing them as
            # one shape would report the wrong ceiling.
            for i, vol in enumerate(z.get("geometry") or []):
                hp = vol.get("horizontalProjection") or {}
                kind = hp.get("type")
                if kind == "Circle":
                    centre = hp.get("center") or []
                    if len(centre) != 2:
                        continue
                    ring = _circle_ring(float(centre[0]), float(centre[1]),
                                        float(hp.get("radius") or 0.0))
                    radius = hp.get("radius")
                elif kind == "Polygon":
                    coords = hp.get("coordinates") or []
                    if not coords or len(coords[0]) < 4:
                        continue
                    ring = [[float(c[0]), float(c[1])] for c in coords[0]]
                    if ring[0] != ring[-1]:
                        ring.append(list(ring[0]))
                    radius = None
                else:
                    continue

                lons = [p[0] for p in ring]
                lats = [p[1] for p in ring]
                out.append({
                    "type": "Feature",
                    "geometry": {"type": "Polygon", "coordinates": [ring]},
                    "bbox": [min(lons), min(lats), max(lons), max(lats)],
                    "properties": {
                        "zone_id": f"{z.get('identifier')}#{i}",
                        "identifier": z.get("identifier"),
                        "name": z.get("name"),
                        "restriction": z.get("restriction"),
                        "reason": z.get("reason") or [],
                        "message": (z.get("message") or "").strip() or None,
                        "lower_limit": vol.get("lowerLimit"),
                        "upper_limit": vol.get("upperLimit"),
                        "vertical_reference": vol.get("upperVerticalReference"),
                        "permanent": any(a.get("permanent") == "YES"
                                         for a in (z.get("applicability") or [])),
                        "circle_radius_m": radius,
                        "authority": _authority(z),
                    },
                })
        _cache = out
        return _cache


def feature_collection(bbox: dict | None = None,
                       restriction: str | None = None) -> dict:
    """Zones as a GeoJSON FeatureCollection, optionally filtered.

    `bbox` is the same {minLon,minLat,maxLon,maxLat} shape the bay reads use.
    Filtering is a cheap bbox-vs-bbox overlap rather than a real intersection:
    it can only ever return a zone the caller did not need, never hide one that
    matters, and the map is going to clip it anyway.
    """
    feats = _features()
    if restriction:
        feats = [f for f in feats if f["properties"]["restriction"] == restriction]
    if bbox:
        lo_x, lo_y = bbox["minLon"], bbox["minLat"]
        hi_x, hi_y = bbox["maxLon"], bbox["maxLat"]
        feats = [f for f in feats
                 if not (f["bbox"][0] > hi_x or f["bbox"][2] < lo_x
                         or f["bbox"][1] > hi_y or f["bbox"][3] < lo_y)]
    return {
        "type": "FeatureCollection",
        "source": os.path.basename(NOFLY_FILE),
        "features": feats,
    }
