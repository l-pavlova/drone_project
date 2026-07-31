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

export type BayStatus = "free" | "occupied" | "unknown";

/** Occupancy TTL: a bay not re-surveyed within this window is "unknown", not free. */
export const FRESHNESS_MS = 10 * 60 * 1000;

export function bayStatus(p: BayProps, now = Date.now()): BayStatus {
  if (p.occupied === null || p.updated_at === null) return "unknown";
  if (now - new Date(p.updated_at).getTime() > FRESHNESS_MS) return "unknown";
  return p.occupied ? "occupied" : "free";
}
