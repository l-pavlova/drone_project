import type { Metrics } from "../lib/types";

export async function fetchMetrics(windowS: number, signal?: AbortSignal): Promise<Metrics> {
  const res = await fetch(`/api/v1/metrics?window_s=${windowS}`, { signal });
  if (!res.ok) throw new Error(`GET /metrics -> ${res.status}`);
  return res.json();
}
