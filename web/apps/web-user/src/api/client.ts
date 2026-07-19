import type { BayFC, ZoneSummary } from "../lib/types";

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
