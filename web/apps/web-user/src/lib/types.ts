export interface BayProps {
  bay_id: string;
  zona: string | null;
  street: string | null;
  park_txt: string | null;
  bearing_deg: number | null;
  public: boolean;
  occupied: boolean | null;
  confidence: number | null;
  last_frame: number | null;
  updated_at: string | null;
  source: string | null;
  /** Which model decided this bay: "heuristic" (the calibrated colour
   *  classifier, simulated surveys) or "detector" (the fine-tuned car detector,
   *  real footage). Null on rows written before migration 0010, and on any
   *  verdict that has aged out of the freshness window. */
  backend: string | null;
  /** Street closure (migration 0012): an EXTERNAL authoritative fact —
   *  roadworks, a market, an accident — that overrides what the camera saw. An
   *  empty bay on a closed street is not available parking. Absent on a server
   *  older than 0012, which is why it is optional rather than `boolean`. */
  closed?: boolean;
  closure?: ClosureInfo | null;
}

/** The closure covering a bay, for the popup. */
export interface ClosureInfo {
  closure_id: string;
  label: string | null;
  reason: string | null;
  valid_to: string | null;
}

export interface BayFeature {
  type: "Feature";
  geometry: { type: "Polygon"; coordinates: number[][][] };
  properties: BayProps;
}

export interface BayFC {
  type: "FeatureCollection";
  features: BayFeature[];
}

export interface ZoneSummary {
  zona: string | null;
  free: number;
  occupied: number;
  unknown: number;
}

/** GET /api/v1/route — road route from the driver to a bay (server proxies OSRM). */
export interface RouteResult {
  distance_m: number;
  duration_s: number;
  coordinates: [number, number][]; // [lon, lat], GeoJSON order
  provider: string;
  cached: boolean;
}

export type BayStatus = "free" | "occupied" | "unknown" | "closed";

/** Occupancy TTL: a bay not re-surveyed within this window is "unknown", not free.
 *
 *  **The SERVER owns this window** (`OCCUPANCY_WINDOW_S`, default 2 h): it is
 *  the same number the occupancy vote runs over, and `/api/v1/bays` has already
 *  NULLed `occupied` on anything past it. A second, shorter TTL here was a bug,
 *  not a belt-and-braces: at a hard-coded 10 minutes the browser greyed out bays
 *  the API was still reporting, so a survey went blank ten minutes after it
 *  landed while the server considered it current for two hours.
 *
 *  The fallback below only matters before the first `/summary` response arrives.
 *  The client still applies it rather than trusting `occupied` alone, because a
 *  tab left open overnight would otherwise keep painting last night's map. */
export const FRESHNESS_FALLBACK_MS = 2 * 60 * 60 * 1000;

let freshnessMs = FRESHNESS_FALLBACK_MS;

/** Adopt the server's occupancy window (seconds), so the two cannot disagree. */
export function setFreshnessWindow(seconds: number): void {
  if (Number.isFinite(seconds) && seconds > 0) freshnessMs = seconds * 1000;
}

export function getFreshnessMs(): number {
  return freshnessMs;
}

export function bayStatus(p: BayProps, now = Date.now()): BayStatus {
  // The closure wins, and it wins FIRST — over "occupied", and over "unknown"
  // too. It is asserted by an authority rather than observed by the drone, so
  // unlike a verdict it does not go stale when the survey does: it ends when
  // its own validity window ends. Checking it after the freshness test would
  // hide a live closure behind an expired observation.
  if (p.closed) return "closed";
  if (p.occupied === null || p.updated_at === null) return "unknown";
  if (now - new Date(p.updated_at).getTime() > freshnessMs) return "unknown";
  return p.occupied ? "occupied" : "free";
}

/** GET /api/v1/nofly — published UAS geographical zones (ED-269, Bulgarian CAA).
 *  Reference data, not live state: it changes when the CAA republishes. */
export type Restriction = "PROHIBITED" | "REQ_AUTHORISATION" | "CONDITIONAL";

export interface NoFlyProps {
  zone_id: string;
  identifier: string | null;
  name: string | null;
  restriction: Restriction;
  reason: string[];
  message: string | null;
  lower_limit: number | null;
  upper_limit: number | null;
  vertical_reference: string | null;
  permanent: boolean;
  /** Set when the source geometry was a circle; the polygon is its 48-gon. */
  circle_radius_m: number | null;
  authority: {
    name: string | null;
    email: string | null;
    phone: string | null;
    interval_before: string | null;
  } | null;
}

export interface NoFlyFeature {
  type: "Feature";
  geometry: { type: "Polygon"; coordinates: number[][][] };
  properties: NoFlyProps;
}

export interface NoFlyFC {
  type: "FeatureCollection";
  /** Filename of the CAA edition this came from, e.g. bgr_zones_30072026.json. */
  source: string;
  features: NoFlyFeature[];
}

/** The drone cannot fly here at all; everything else is paperwork, not a wall. */
export function blocksSurvey(p: NoFlyProps): boolean {
  return p.restriction === "PROHIBITED";
}
