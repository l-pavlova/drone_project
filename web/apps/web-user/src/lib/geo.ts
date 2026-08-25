import type { BayFeature, BayProps, BayStatus } from "./types";

export interface UserPos {
  lat: number;
  lon: number;
  accuracy: number;
}

/** Default you-are-here: on бул. Джеймс Баучер, in front of FMI. Used until real
 *  geolocation succeeds, so nearest-free works from the block out of the box.
 *
 *  It is a point ON THE STREET, not the block centroid — a driver arrives along
 *  the boulevard, and a default sitting on the bay cluster itself makes the
 *  distances and the route look wrong. Derived by projecting the FMI address
 *  geocode (42.6743496, 23.3305178) perpendicularly onto the Bourchier
 *  centerline in data/block_roads.geojson; 35 m from the ENU origin, so it stays
 *  well inside the surveyed fmi_block area.
 *
 *  UI-only. It is NOT the ENU origin, despite once sharing its value: that
 *  constant lives in packages/contracts and must stay in lockstep with
 *  generate_world.py / score_occupancy.py. */
export const FMI_DEFAULT: UserPos = { lat: 42.674992, lon: 23.330087, accuracy: 25 };

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

/** Continuous geolocation, used while navigating so the route follows the
 *  driver. Returns an unsubscribe that clears the watch. */
export function watchPosition(
  onPos: (p: UserPos) => void,
  onError?: (e: Error) => void,
): () => void {
  if (!navigator.geolocation) {
    onError?.(new Error("geolocation not supported"));
    return () => {};
  }
  const id = navigator.geolocation.watchPosition(
    (p) =>
      onPos({
        lat: p.coords.latitude,
        lon: p.coords.longitude,
        accuracy: p.coords.accuracy,
      }),
    (err) => onError?.(new Error(err.message || "location denied")),
    { enableHighAccuracy: true, timeout: 15_000, maximumAge: 5_000 },
  );
  return () => navigator.geolocation.clearWatch(id);
}

/** Google Maps driving-directions deep link — opens the native Maps app on
 *  mobile, google.com/maps on desktop. Our hand-off for actual navigation. */
export function directionsUrl(
  from: { lat: number; lon: number } | null,
  to: { lat: number; lon: number },
): string {
  const q = new URLSearchParams({
    api: "1",
    destination: `${to.lat},${to.lon}`,
    travelmode: "driving",
  });
  if (from) q.set("origin", `${from.lat},${from.lon}`);
  return `https://www.google.com/maps/dir/?${q.toString()}`;
}

export function formatDistance(m: number): string {
  return m < 950 ? `${Math.round(m / 10) * 10} m` : `${(m / 1000).toFixed(1)} km`;
}

export function formatDuration(s: number): string {
  const min = Math.round(s / 60);
  if (min < 1) return "< 1 min";
  if (min < 60) return `~${min} min`;
  return `~${Math.floor(min / 60)} h ${min % 60} min`;
}

/** Nearest free bay to a position, using the merged (live) occupancy state. */
export function nearestFree(
  features: BayFeature[],
  // BayStatus rather than an inline union: this predicate keeps only "free", so
  // a new status must be added in one place and is excluded here by default —
  // which is the safe direction. A closed bay is never offered for parking.
  statusOf: (p: BayProps) => BayStatus,
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
