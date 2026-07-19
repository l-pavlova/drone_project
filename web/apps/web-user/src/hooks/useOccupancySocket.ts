import { useEffect, useState } from "react";

export interface BayDelta {
  type: "bay_delta";
  bay_id: string;
  occupied: boolean;
  confidence: number;
  updated_at: string;
}

export interface LiveState {
  occupied: boolean;
  confidence: number;
  updated_at: string;
}

/**
 * Subscribe to /ws/occupancy and maintain a map of live bay-state overrides.
 * Auto-reconnects with backoff. Returns the overlay map + connection status;
 * the caller merges these onto the initial /bays snapshot.
 */
export function useOccupancySocket(): {
  live: Map<string, LiveState>;
  connected: boolean;
} {
  const [connected, setConnected] = useState(false);
  // A NEW Map on every delta so memoized consumers (readout counts, bay colors,
  // open popups) re-render. A mutated ref would keep the same identity and leave
  // React.memo / useMemo consumers stale.
  const [live, setLive] = useState<Map<string, LiveState>>(() => new Map());

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closed = false;
    let backoff = 500;

    const connect = () => {
      const proto = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${proto}://${location.host}/ws/occupancy`);

      ws.onopen = () => {
        setConnected(true);
        backoff = 500;
      };
      ws.onmessage = (ev) => {
        let msg: unknown;
        try {
          msg = JSON.parse(ev.data as string);
        } catch {
          return;
        }
        const d = msg as BayDelta;
        if (d?.type === "bay_delta") {
          setLive((prev) => {
            const next = new Map(prev);
            next.set(d.bay_id, {
              occupied: d.occupied,
              confidence: d.confidence,
              updated_at: d.updated_at,
            });
            return next;
          });
        }
      };
      ws.onclose = () => {
        setConnected(false);
        if (!closed) {
          setTimeout(connect, backoff);
          backoff = Math.min(backoff * 2, 10_000);
        }
      };
      ws.onerror = () => ws?.close();
    };

    connect();
    return () => {
      closed = true;
      ws?.close();
    };
  }, []);

  return { live, connected };
}
