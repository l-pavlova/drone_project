import { useEffect, useRef, useState } from "react";

export interface BayDelta {
  type: "bay_delta";
  cursor?: string;
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
  syncVersion: number;
} {
  const [connected, setConnected] = useState(false);
  const [syncVersion, setSyncVersion] = useState(0);
  // A NEW Map on every delta so memoized consumers (readout counts, bay colors,
  // open popups) re-render. A mutated ref would keep the same identity and leave
  // React.memo / useMemo consumers stale.
  const [live, setLive] = useState<Map<string, LiveState>>(() => new Map());
  // Keep this outside React state: receiving a cursor is transport book-keeping
  // rather than a UI update. It survives each reconnect within this page load.
  const cursor = useRef<string | null>(null);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closed = false;
    let backoff = 500;

    const connect = () => {
      const proto = location.protocol === "https:" ? "wss" : "ws";
      const since = cursor.current ? `?since=${encodeURIComponent(cursor.current)}` : "";
      ws = new WebSocket(`${proto}://${location.host}/ws/occupancy${since}`);

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
          if (d.cursor) cursor.current = d.cursor;
          setLive((prev) => {
            const next = new Map(prev);
            next.set(d.bay_id, {
              occupied: d.occupied,
              confidence: d.confidence,
              updated_at: d.updated_at,
            });
            return next;
          });
        } else if (
          typeof msg === "object" &&
          msg !== null &&
          (msg as { type?: string; cursor?: unknown }).type === "snapshot_cursor" &&
          typeof (msg as { cursor?: unknown }).cursor === "string"
        ) {
          // Sent after replay, so this is the newest safe resume point.
          const nextCursor = (msg as { cursor: string }).cursor;
          // Cursors are process-local. If the API restarted, its counter goes
          // back to zero; discard the old-process overlay before reconciling
          // from REST, otherwise it could mask fresher DB state indefinitely.
          if (
            cursor.current !== null &&
            Number.isFinite(Number(cursor.current)) &&
            Number.isFinite(Number(nextCursor)) &&
            Number(nextCursor) < Number(cursor.current)
          ) {
            setLive(new Map());
          }
          cursor.current = nextCursor;
          // Tell the app that a complete replay boundary has been reached. It
          // can refresh its canonical REST snapshot once, while `live` remains
          // the authoritative overlay for any delta arriving during that fetch.
          setSyncVersion((n) => n + 1);
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

  return { live, connected, syncVersion };
}
