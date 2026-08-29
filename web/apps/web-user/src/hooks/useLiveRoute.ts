import { useEffect, useRef, useState } from "react";
import { fetchRoute } from "../api/client";
import { haversine, type NearestTarget, type UserPos } from "../lib/geo";
import type { RouteResult } from "../lib/types";

/** Re-route once the driver has moved this far from the position the current
 *  route was computed for. Above the server's ~11 m cache quantum, so a fix
 *  that does clear this threshold is a genuine upstream lookup. */
const REROUTE_MOVE_M = 25;

export type RouteStatus = "idle" | "loading" | "ready" | "error";

/**
 * Keeps a road route from the driver to the chosen spot, refetched as they move.
 *
 * `userPos` changes identity on every GPS fix, so this effect runs constantly
 * and bails out early unless the target changed or the driver actually travelled
 * REROUTE_MOVE_M. On a re-route the previous line stays on screen (no blink);
 * only a new target clears it.
 */
export function useLiveRoute(
  userPos: UserPos | null,
  target: NearestTarget | null,
): { route: RouteResult | null; status: RouteStatus } {
  const [route, setRoute] = useState<RouteResult | null>(null);
  const [status, setStatus] = useState<RouteStatus>("idle");
  // position + target the current (or in-flight) route was computed for. Keyed
  // on the target ID rather than a bay id: the destination can be a kerb gap.
  const anchor = useRef<{ lat: number; lon: number; id: string } | null>(null);
  const inflight = useRef<AbortController | null>(null);

  useEffect(() => {
    if (!target || !userPos) {
      inflight.current?.abort();
      inflight.current = null;
      anchor.current = null;
      setRoute(null);
      setStatus("idle");
      return;
    }

    const a = anchor.current;
    const sameTarget = a?.id === target.id;
    if (sameTarget && haversine(a!.lat, a!.lon, userPos.lat, userPos.lon) < REROUTE_MOVE_M) {
      return;
    }

    anchor.current = { lat: userPos.lat, lon: userPos.lon, id: target.id };
    inflight.current?.abort();
    const ctl = new AbortController();
    inflight.current = ctl;

    if (!sameTarget) setRoute(null);
    setStatus("loading");

    fetchRoute(userPos, target, ctl.signal)
      .then((r) => {
        if (ctl.signal.aborted) return;
        setRoute(r);
        setStatus("ready");
      })
      .catch((e: unknown) => {
        if (ctl.signal.aborted || (e as Error)?.name === "AbortError") return;
        // router unavailable (502) or offline: drop to the straight-line hint
        setRoute(null);
        setStatus("error");
      });
  }, [userPos, target]);

  // abort a route still in flight when the map unmounts
  useEffect(() => () => inflight.current?.abort(), []);

  return { route, status };
}
