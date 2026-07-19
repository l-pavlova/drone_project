import type { BayFeature, BayProps } from "./types";

export interface UserPos {
  lat: number;
  lon: number;
  accuracy: number;
}

/** Default you-are-here: the FMI block centroid (ENU origin). Used until real
 *  geolocation succeeds, so nearest-free works from the block out of the box. */
export const FMI_DEFAULT: UserPos = { lat: 42.6747105, lon: 23.3298956, accuracy: 25 };

export interface NearestTarget {
  bayId: string;
  lat: number;
  lon: number;
  distance: number; // metres
}

/** Great-circle distance in metres between two lon/lat points. */
export function haversine(aLat: number, aLon: number, bLat: number, bLon: number): number {
  const R = 6371000;
  const dLat = ((bLat - aLat) * Math.PI) / 180;
  const dLon = ((bLon - aLon) * Math.PI) / 180;
  const s =
    Math.sin(dLat / 2) ** 2 +
    Math.cos((aLat * Math.PI) / 180) *
      Math.cos((bLat * Math.PI) / 180) *
      Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(s));
}

/** Representative point of a bay (mean of its polygon ring, [lon,lat]). */
export function bayCentroid(f: BayFeature): { lat: number; lon: number } {
  const ring = f.geometry.coordinates[0]!;
  let lon = 0;
  let lat = 0;
  for (const [x, y] of ring) {
    lon += x!;
    lat += y!;
  }
  return { lon: lon / ring.length, lat: lat / ring.length };
}

/** Promisified single geolocation fix. */
export function getPosition(): Promise<UserPos> {
  return new Promise((resolve, reject) => {
    if (!navigator.geolocation) {
      reject(new Error("geolocation not supported"));
      return;
    }
    navigator.geolocation.getCurrentPosition(
      (p) =>
        resolve({
          lat: p.coords.latitude,
          lon: p.coords.longitude,
          accuracy: p.coords.accuracy,
        }),
      (err) => reject(new Error(err.message || "location denied")),
      { enableHighAccuracy: true, timeout: 10_000, maximumAge: 30_000 },
    );
  });
}

/** Nearest free bay to a position, using the merged (live) occupancy state. */
export function nearestFree(
  features: BayFeature[],
  statusOf: (p: BayProps) => "free" | "occupied" | "unknown",
  from: UserPos,
): NearestTarget | null {
  let best: NearestTarget | null = null;
  for (const f of features) {
    if (statusOf(f.properties) !== "free") continue;
    const c = bayCentroid(f);
    const d = haversine(from.lat, from.lon, c.lat, c.lon);
    if (!best || d < best.distance) {
      best = { bayId: f.properties.bay_id, lat: c.lat, lon: c.lon, distance: d };
    }
  }
  return best;
}
