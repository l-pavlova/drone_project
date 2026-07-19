/**
 * Shared local-ENU projection — the "spine" of the PARKDRONE project.
 *
 * These constants MUST stay identical to `sim/generate_world.py` and
 * `vision/score_occupancy.py` (ORIGIN, MLAT, MLON). Bay geometry is stored in
 * WGS84 (EPSG:4326) and never needs this; the projection is only for converting
 * drone poses (ENU metres) to/from lon/lat, e.g. to place a pose on the map or
 * to reason about distances the way the vision stage does.
 */

/** (lat, lon) of the pinned block centroid — the ENU origin. */
export const ORIGIN = { lat: 42.6747105, lon: 23.3298956 } as const;

/** metres per degree latitude (spherical approximation used project-wide). */
export const MLAT = 111320.0;

/** metres per degree longitude at ORIGIN latitude. */
export const MLON = 111320.0 * Math.cos((ORIGIN.lat * Math.PI) / 180);

export interface LonLat {
  lon: number;
  lat: number;
}

export interface Enu {
  /** East metres. */
  x: number;
  /** North metres. */
  y: number;
}

/** lon/lat (WGS84) -> local ENU metres. Mirrors `to_enu` in score_occupancy.py. */
export function toEnu(p: LonLat): Enu {
  return {
    x: (p.lon - ORIGIN.lon) * MLON,
    y: (p.lat - ORIGIN.lat) * MLAT,
  };
}

/** local ENU metres -> lon/lat (WGS84). Inverse of {@link toEnu}. */
export function toLonLat(e: Enu): LonLat {
  return {
    lon: ORIGIN.lon + e.x / MLON,
    lat: ORIGIN.lat + e.y / MLAT,
  };
}
