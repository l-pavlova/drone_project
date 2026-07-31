"""Driving-route lookup behind `GET /api/v1/route` (directions to a free bay).

The browser never calls the routing provider directly — the server proxies it, so

  * the user's coordinates stay on our own origin,
  * the provider is swappable (public OSRM demo in dev; a self-hosted OSRM over
    the same OSM extract the project already downloads for
    `data/block_roads.geojson` in production) with no frontend change,
  * and a small TTL cache absorbs the repeated re-routes the map fires while the
    driver is moving.

Uses `urllib` rather than `requests`, matching the data scripts — no new
dependency. Verification is on by default; if Windows' flaky cert revocation
trips it (the same reason `tools/*.py` disable it for public GETs) we retry once
without verification and say so, instead of failing the request.
"""
import json
import ssl
import threading
import time
import urllib.error
import urllib.request

from .config import OSRM_BASE, OSRM_TIMEOUT_S, ROUTE_CACHE_MAX, ROUTE_CACHE_TTL_S


class RoutingError(RuntimeError):
    """Upstream router unreachable, or no road route between the two points."""


# Cache-key quantum in decimal degrees (4 dp ~ 11 m). OSRM snaps both endpoints
# to the nearest road anyway, so rounding the origin costs nothing visible and
# collapses a driver's stream of GPS fixes onto one key.
_QUANT = 4

_cache: "dict[tuple, tuple[float, dict]]" = {}
_lock = threading.Lock()
_insecure_warned = False


def _key(flon: float, flat: float, tlon: float, tlat: float) -> tuple:
    return (
        round(flon, _QUANT), round(flat, _QUANT),
        round(tlon, _QUANT), round(tlat, _QUANT),
    )


def _cache_get(k: tuple):
    now = time.time()
    with _lock:
        hit = _cache.get(k)
        if hit is None:
            return None
        if hit[0] > now:
            return hit[1]
        del _cache[k]  # expired
    return None


def _cache_put(k: tuple, payload: dict) -> None:
    now = time.time()
    with _lock:
        if len(_cache) >= ROUTE_CACHE_MAX:
            for dead in [kk for kk, (exp, _) in _cache.items() if exp <= now]:
                del _cache[dead]
            while len(_cache) >= ROUTE_CACHE_MAX:  # still full: drop oldest
                _cache.pop(next(iter(_cache)))
        _cache[k] = (now + ROUTE_CACHE_TTL_S, payload)


def _get_json(url: str) -> dict:
    global _insecure_warned
    try:
        with urllib.request.urlopen(url, timeout=OSRM_TIMEOUT_S) as r:
            return json.loads(r.read().decode("utf-8"))
    except (urllib.error.URLError, ssl.SSLError) as exc:
        # urllib wraps the TLS failure in URLError, so the SSLError is in .reason
        cause = getattr(exc, "reason", exc)
        if not isinstance(cause, ssl.SSLError):
            raise
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        if not _insecure_warned:
            print(f"! routing: TLS verification failed ({cause}); retrying unverified")
            _insecure_warned = True
        with urllib.request.urlopen(url, timeout=OSRM_TIMEOUT_S, context=ctx) as r:
            return json.loads(r.read().decode("utf-8"))


def driving_route(flon: float, flat: float, tlon: float, tlat: float) -> dict:
    """Road route between two lon/lat points.

    Returns {distance_m, duration_s, coordinates: [[lon,lat], ...], provider,
    cached}; coordinates are GeoJSON order, like everything else we serve.
    Raises RoutingError if the router fails or finds no route (the map then falls
    back to its straight-line hint).
    """
    k = _key(flon, flat, tlon, tlat)
    cached = _cache_get(k)
    if cached is not None:
        return {**cached, "cached": True}

    url = (
        f"{OSRM_BASE.rstrip('/')}/route/v1/driving/"
        f"{k[0]},{k[1]};{k[2]},{k[3]}"
        "?overview=full&geometries=geojson&alternatives=false&steps=false"
    )
    try:
        doc = _get_json(url)
    except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
        raise RoutingError(f"router unreachable: {exc}") from exc

    if doc.get("code") != "Ok" or not doc.get("routes"):
        raise RoutingError(f"no route ({doc.get('code', 'unknown')})")

    r0 = doc["routes"][0]
    try:
        payload = {
            "distance_m": float(r0["distance"]),
            "duration_s": float(r0["duration"]),
            "coordinates": r0["geometry"]["coordinates"],
            "provider": "osrm",
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise RoutingError(f"malformed router response: {exc}") from exc

    _cache_put(k, payload)
    return {**payload, "cached": False}
