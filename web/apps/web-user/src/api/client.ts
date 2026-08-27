import type { BayFC, NoFlyFC, RouteResult, ZoneSummary, CurbRunFC } from "../lib/types";

export async function fetchBays(bbox?: string, zona?: string): Promise<BayFC> {
  const q = new URLSearchParams();
  if (bbox) q.set("bbox", bbox);
  if (zona) q.set("zona", zona);
  const res = await fetch(`/api/v1/bays?${q.toString()}`);
  if (!res.ok) throw new Error(`GET /bays -> ${res.status}`);
  return res.json();
}

/** Server-side occupancy window + active backend. Fetched once at startup so
 *  the client's staleness TTL is the server's, not a second hard-coded guess
 *  that can drift out of step with the vote (it used to, by 12x). */
export async function fetchServerInfo(): Promise<{
  ok: boolean;
  occupancy_window_s: number;
  occupancy_backend: string;
}> {
  const res = await fetch("/health");
  if (!res.ok) throw new Error(`GET /health -> ${res.status}`);
  return res.json();
}

export async function fetchSummary(): Promise<{ zones: ZoneSummary[] }> {
  const res = await fetch("/api/v1/summary");
  if (!res.ok) throw new Error(`GET /summary -> ${res.status}`);
  return res.json();
}

/** Road route between two points. 502 means the router is unavailable — the
 *  caller falls back to the straight-line hint rather than showing an error. */
export async function fetchRoute(
  from: { lat: number; lon: number },
  to: { lat: number; lon: number },
  signal?: AbortSignal,
): Promise<RouteResult> {
  const q = new URLSearchParams({
    from: `${from.lon},${from.lat}`,
    to: `${to.lon},${to.lat}`,
  });
  const res = await fetch(`/api/v1/route?${q.toString()}`, { signal });
  if (!res.ok) throw new Error(`GET /route -> ${res.status}`);
  return res.json();
}

/** Published UAS no-fly / restricted zones for a viewport.
 *  Reference data — fetched once, never pushed over the socket. */
export async function fetchNoFly(bbox?: string): Promise<NoFlyFC> {
  const q = new URLSearchParams();
  if (bbox) q.set("bbox", bbox);
  const res = await fetch(`/api/v1/nofly?${q.toString()}`);
  if (!res.ok) throw new Error(`GET /nofly -> ${res.status}`);
  return res.json();
}

/** Curb runs with their current free-space count (migration 0014).
 *
 *  Polled rather than pushed: the WebSocket delta channel carries bay_id
 *  payloads, and widening it to runs would mean a second delta table plus a
 *  listener change for a layer that changes at survey speed. A short poll is the
 *  smaller correct thing here.
 */
export async function fetchCurbRuns(bbox?: string): Promise<CurbRunFC> {
  const q = new URLSearchParams();
  if (bbox) q.set("bbox", bbox);
  const res = await fetch(`/api/v1/runs?${q.toString()}`);
  if (!res.ok) throw new Error(`GET /runs -> ${res.status}`);
  return res.json();
}
