import type { BayFeature, BayProps, BayStatus, CurbRunFC } from "./types";

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

/** Somewhere the driver can be sent to park.
 *
 *  Two kinds, because the map now publishes two incompatible geometries for the
 *  same question: a published bay RECTANGLE, and a measured GAP along a kerb.
 *  Which one the FAB aims at follows the readout's source selector — see
 *  `CountSource` in types.ts. */
export interface NearestTarget {
  kind: "bay" | "gap";
  /** Stable identity: a bay_id, or `${run_id}@${s0}` for a gap. `useLiveRoute`
   *  anchors on this and must not refetch a route while it is unchanged, so it
   *  has to be stable across polls — which is why a gap keys off its start
   *  arclength rather than its index in the `gaps` array. */
  id: string;
  /** What the route toast says: "bay 17596" / "kerb on Бургас". */
  label: string;
  /** kind === "gap" only: which run, and where along it, so the re-target effect
   *  can ask whether that stretch is still free on the next poll. */
  runId?: string;
  s?: number;
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
      const id = f.properties.bay_id;
      best = { kind: "bay", id, label: `bay ${id}`, lat: c.lat, lon: c.lon, distance: d };
    }
  }
  return best;
}

// ---- kerb arclength geometry -----------------------------------------------
// Shared with CurbRunLayer, which DRAWS the same gaps this routes to. One walk,
// deliberately: if the two diverged, the marker would sit off the bright segment
// the driver is looking at, and neither would be visibly wrong on its own.

/** A polyline vertex in leaflet order, [lat, lon]. */
export type LatLon = [number, number];

/** The stretch of a run's polyline between two arclengths.
 *
 *  Gaps are reported in the run's own arclength metres (the server measures them
 *  along the same polyline it publishes), so cutting one out is a matter of
 *  walking the line and splitting it at s0 and s1 — including the partial
 *  segment at each end, or a short gap inside one long segment would render as
 *  nothing at all.
 *
 *  Distances are haversine where the server used ENU metres. The two differ by
 *  well under 0.1% at this latitude and block scale — under 10 cm on an 80 m
 *  run, far below a pixel at any zoom this map offers — and the alternative is a
 *  FIFTH copy of the ORIGIN/MLAT/MLON constants (generate_world.py,
 *  score_occupancy.py, packages/contracts, the sim controller) whose drift would
 *  be a real bug. Position is all that is taken from this; every number shown to
 *  the user is the server's.
 */
export function sliceByArclength(positions: LatLon[], s0: number, s1: number): LatLon[] {
  const out: LatLon[] = [];
  let acc = 0;
  for (let i = 0; i < positions.length - 1; i++) {
    const a = positions[i];
    const b = positions[i + 1];
    if (!a || !b) continue;
    const [aLat, aLon] = a;
    const [bLat, bLon] = b;
    const seg = haversine(aLat, aLon, bLat, bLon);
    if (seg <= 0) continue;
    const segEnd = acc + seg;
    if (segEnd > s0 && acc < s1) {
      const t0 = Math.max(0, (s0 - acc) / seg);
      const t1 = Math.min(1, (s1 - acc) / seg);
      const at = (t: number): LatLon => [aLat + (bLat - aLat) * t, aLon + (bLon - aLon) * t];
      if (out.length === 0) out.push(at(t0));
      out.push(at(t1));
    }
    acc = segEnd;
  }
  return out;
}

/** The single point `s` metres along a run's polyline, clamped to its ends.
 *
 *  Same walk as `sliceByArclength` — kept beside it so the point this routes to
 *  is always on the segment that gets drawn. */
export function pointAtArclength(
  positions: LatLon[],
  s: number,
): { lat: number; lon: number } | null {
  if (positions.length === 0) return null;
  const first = positions[0]!;
  if (s <= 0) return { lat: first[0], lon: first[1] };
  let acc = 0;
  for (let i = 0; i < positions.length - 1; i++) {
    const a = positions[i];
    const b = positions[i + 1];
    if (!a || !b) continue;
    const [aLat, aLon] = a;
    const [bLat, bLon] = b;
    const seg = haversine(aLat, aLon, bLat, bLon);
    if (seg <= 0) continue;
    if (acc + seg >= s) {
      const t = (s - acc) / seg;
      return { lat: aLat + (bLat - aLat) * t, lon: aLon + (bLon - aLon) * t };
    }
    acc += seg;
  }
  const last = positions[positions.length - 1]!;
  return { lat: last[0], lon: last[1] };
}

/** Nearest measured free GAP along a kerb — the run layer's answer to
 *  `nearestFree`.
 *
 *  A gap is a stretch the survey measured as both observed and unoccupied, and
 *  `spaces` is how many cars fit in it after manoeuvring clearance. Only gaps
 *  that actually hold a car are offered: a 3 m gap is real kerb and is worth
 *  drawing, but sending someone to it is worse than saying nothing.
 *
 *  Aims at the gap's MIDPOINT rather than its start, so the driver arrives in
 *  the middle of the free stretch rather than at the bumper of the car bounding
 *  it. */
export function nearestFreeGap(runs: CurbRunFC | null, from: UserPos): NearestTarget | null {
  if (!runs) return null;
  let best: NearestTarget | null = null;
  for (const f of runs.features) {
    const p = f.properties;
    if (!p.gaps || p.gaps.length === 0) continue;
    const positions = f.geometry.coordinates.map(([lon, lat]) => [lat, lon] as LatLon);
    if (positions.length < 2) continue;
    for (const g of p.gaps) {
      if (g.spaces <= 0) continue;
      const s = (g.s0 + g.s1) / 2;
      const pt = pointAtArclength(positions, s);
      if (!pt) continue;
      const d = haversine(from.lat, from.lon, pt.lat, pt.lon);
      if (!best || d < best.distance) {
        best = {
          kind: "gap",
          id: `${p.run_id}@${g.s0}`,
          label: p.street ? `kerb on ${p.street}` : "kerb space",
          runId: p.run_id,
          s,
          lat: pt.lat,
          lon: pt.lon,
          distance: d,
        };
      }
    }
  }
  return best;
}

/** Is the stretch this target sits on still published as free?
 *
 *  The kerb counterpart of re-checking `bayStatus` on the bay the driver is
 *  driving to. The run layer is POLLED rather than pushed, so this runs on every
 *  refresh: a gap that has since been parked in simply stops being published,
 *  and the driver gets re-routed instead of arriving at a taken space. */
export function gapStillFree(runs: CurbRunFC | null, target: NearestTarget): boolean {
  if (target.kind !== "gap" || target.runId == null || target.s == null) return false;
  // No fresh poll yet: keep the target rather than cancelling a live route on
  // the strength of data we do not have.
  if (!runs) return true;
  const f = runs.features.find((x) => x.properties.run_id === target.runId);
  if (!f) return true;
  const s = target.s;
  return (f.properties.gaps ?? []).some((g) => g.spaces > 0 && g.s0 <= s && s <= g.s1);
}
