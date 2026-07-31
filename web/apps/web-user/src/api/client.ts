import type { BayFC, RouteResult, ZoneSummary } from "../lib/types";

export async function fetchBays(bbox?: string, zona?: string): Promise<BayFC> {
  const q = new URLSearchParams();
  if (bbox) q.set("bbox", bbox);
  if (zona) q.set("zona", zona);
  const res = await fetch(`/api/v1/bays?${q.toString()}`);
  if (!res.ok) throw new Error(`GET /bays -> ${res.status}`);
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
