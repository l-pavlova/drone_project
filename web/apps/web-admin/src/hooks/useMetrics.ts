import { useCallback, useEffect, useRef, useState } from "react";
import { fetchMetrics } from "../api/client";
import type { Metrics } from "../lib/types";

export interface Sample {
  t: number; // ms epoch
  framesPerMin: number;
  queueDepth: number;
}

const HISTORY_MAX = 180; // ~9 min at the 3 s default

/** Polls GET /api/v1/metrics and keeps a short rolling history.
 *
 *  Polling, not WebSocket: these are gauges read from Postgres, and the socket
 *  carries bay deltas, not pipeline state. The history is accumulated CLIENT
 *  SIDE from the polls — the endpoint returns instantaneous figures, so the
 *  sparkline only knows what it has watched since the page opened, and the UI
 *  says so rather than implying a stored series.
 *
 *  A failed poll keeps the last good snapshot on screen and surfaces `error`;
 *  blanking a dashboard because one request timed out loses the very context
 *  you need to tell "the pipeline died" from "my laptop slept". */
export function useMetrics(windowS: number, intervalMs = 3000) {
  const [data, setData] = useState<Metrics | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [history, setHistory] = useState<Sample[]>([]);
  const [lastOk, setLastOk] = useState<number | null>(null);
  const inFlight = useRef<AbortController | null>(null);

  const poll = useCallback(async () => {
    inFlight.current?.abort();
    const ctrl = new AbortController();
    inFlight.current = ctrl;
    try {
      const m = await fetchMetrics(windowS, ctrl.signal);
      setData(m);
      setError(null);
      setLastOk(Date.now());
      setHistory((h) =>
        [
          ...h,
          { t: Date.now(), framesPerMin: m.ingest.frames_last_1m, queueDepth: m.queue.depth },
        ].slice(-HISTORY_MAX),
      );
    } catch (e) {
      if ((e as Error).name === "AbortError") return;
      setError((e as Error).message);
    }
  }, [windowS]);

  useEffect(() => {
    void poll();
    const id = setInterval(() => void poll(), intervalMs);
    return () => {
      clearInterval(id);
      inFlight.current?.abort();
    };
  }, [poll, intervalMs]);

  return { data, error, history, lastOk, refresh: poll };
}
