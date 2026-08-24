import type { Tone } from "./types";

const DASH = "—";

/** Seconds as a compact age: 8s, 4m 12s, 2h 05m. */
export function age(s: number | null | undefined): string {
  if (s == null) return DASH;
  if (s < 60) return `${Math.round(s)}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${String(Math.round(s % 60)).padStart(2, "0")}s`;
  return `${Math.floor(s / 3600)}h ${String(Math.floor((s % 3600) / 60)).padStart(2, "0")}m`;
}

/** Sub-second latencies read better in ms; keep seconds once they get big. */
export function seconds(s: number | null | undefined): string {
  if (s == null) return DASH;
  if (s < 1) return `${Math.round(s * 1000)} ms`;
  if (s < 60) return `${s.toFixed(1)} s`;
  return age(s);
}

export function count(n: number | null | undefined): string {
  return n == null ? DASH : n.toLocaleString("en-US");
}

export function percent(fraction: number | null | undefined, digits = 1): string {
  return fraction == null ? DASH : `${(fraction * 100).toFixed(digits)}%`;
}

export function clock(iso: string | null): string {
  return iso ? new Date(iso).toLocaleTimeString() : DASH;
}

/** Seconds since an ISO timestamp, or null if there isn't one. */
export function secondsSince(iso: string | null): number | null {
  return iso ? (Date.now() - new Date(iso).getTime()) / 1000 : null;
}

// ---- health rules -----------------------------------------------------------
//
// One place for every threshold, so the tiles, the fleet table and the banner
// can never disagree about what "stalled" means.

export const STALL_WARN_S = 60;
export const STALL_CRIT_S = 300;
export const DEPTH_WARN = 50;
export const MISSION_IDLE_WARN_S = 120;
/** Lapsed leases are normal in ones — a replica restarting mid-frame leaves a
 *  few and the next claim takes them over within a poll. A pile of them means
 *  something is dying repeatedly. */
export const EXPIRED_LEASE_WARN = 5;

/** A backlog is only a problem if it isn't moving: depth alone is a burst, but
 *  a job that has sat UNCLAIMED for minutes means no replica is draining it.
 *  `depth` here is the shared backlog (jobs.queued), not one replica's local
 *  prefetch — with several replicas the latter says nothing about the whole. */
export function queueTone(depth: number, oldestQueuedS: number | null): Tone {
  if (oldestQueuedS != null && oldestQueuedS >= STALL_CRIT_S) return "critical";
  if (oldestQueuedS != null && oldestQueuedS >= STALL_WARN_S) return "warn";
  if (depth >= DEPTH_WARN) return "warn";
  return depth > 0 ? "good" : "idle";
}

/** Lapsed leases: fine in ones, a symptom in numbers. */
export function leaseTone(expired: number): Tone {
  if (expired >= EXPIRED_LEASE_WARN) return "warn";
  return expired > 0 ? "good" : "idle";
}

export function failureTone(rate: number | null): Tone {
  if (rate == null) return "idle";
  if (rate >= 0.1) return "critical";
  return rate > 0 ? "warn" : "good";
}

/** Accuracy bands. Deliberately strict: this classifier scores 100% on both
 *  held-out sim worlds today, so anything under ~95% is a real regression, not
 *  noise. No ground truth (production) is "idle" — unknown, not bad. */
export function accuracyTone(acc: number | null): Tone {
  if (acc == null) return "idle";
  if (acc < 0.9) return "critical";
  if (acc < 0.95) return "warn";
  return "good";
}

/** An idle pipeline is not a broken one — no drone is flying most of the time.
 *  Only a mission that is still open but has gone quiet is worth flagging. */
export function missionTone(sinceLastFrameS: number | null): Tone {
  if (sinceLastFrameS == null) return "idle";
  return sinceLastFrameS >= MISSION_IDLE_WARN_S ? "warn" : "good";
}
