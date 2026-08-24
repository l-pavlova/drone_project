import { useMemo, useState } from "react";
import { CoverageBar } from "../components/CoverageBar";
import { FleetTable } from "../components/FleetTable";
import { Panel } from "../components/Panel";
import { Sparkline } from "../components/Sparkline";
import { Stat, StatGrid } from "../components/Stat";
import { useMetrics } from "../hooks/useMetrics";
import {
  STALL_CRIT_S,
  STALL_WARN_S,
  accuracyTone,
  age,
  clock,
  count,
  failureTone,
  leaseTone,
  percent,
  queueTone,
  seconds,
  secondsSince,
} from "../lib/format";
import type { Tone } from "../lib/types";
import styles from "./App.module.css";

const WINDOWS = [
  { label: "1m", value: 60 },
  { label: "5m", value: 300 },
  { label: "15m", value: 900 },
  { label: "1h", value: 3600 },
];

export default function App() {
  const [windowS, setWindowS] = useState(300);
  const { data, error, history, lastOk } = useMetrics(windowS);

  const spark = useMemo(
    () => history.map((h) => ({ t: h.t, v: h.framesPerMin })),
    [history],
  );

  // The shared backlog, not this replica's prefetch: with job claiming the
  // local queue is deliberately shallow and says nothing about the whole.
  const qTone = data ? queueTone(data.jobs.queued, data.jobs.oldest_queued_age_s) : "idle";
  const fTone = data ? failureTone(data.latency.failure_rate) : "idle";
  const idleS = data ? secondsSince(data.ingest.last_frame_at) : null;

  // One banner, one sentence, only when something is actually wrong. A
  // dashboard that shouts on every idle minute stops being read.
  const alert = buildAlert(qTone, fTone, data?.jobs.oldest_queued_age_s ?? null, error);

  return (
    <div className={styles.page}>
      <header className={styles.top}>
        <div>
          <h1 className={styles.wordmark}>PARKDRONE · OPS</h1>
          <p className={styles.sub}>
            {data
              ? `replica ${data.process.replica_id} · up ${age(data.process.uptime_s)} · ${data.process.classify_threads} classify threads · ${data.process.hub_clients} live client${data.process.hub_clients === 1 ? "" : "s"}`
              : "connecting to the server…"}
          </p>
        </div>

        <div className={styles.controls}>
          <span className={styles.ctrlLabel}>window</span>
          <div className={styles.segmented} role="group" aria-label="Metrics window">
            {WINDOWS.map((w) => (
              <button
                key={w.value}
                type="button"
                className={`${styles.segBtn} ${windowS === w.value ? styles.segOn : ""}`}
                onClick={() => setWindowS(w.value)}
              >
                {w.label}
              </button>
            ))}
          </div>
          <span className={`${styles.pulse} ${error ? styles.pulseBad : styles.pulseOk}`}>
            <span className={styles.beacon} />
            {error ? "no connection" : `polled ${clock(lastOk ? new Date(lastOk).toISOString() : null)}`}
          </span>
        </div>
      </header>

      {alert && (
        <div className={`${styles.banner} ${styles[alert.tone]}`} role="status">
          <strong>{alert.title}</strong> {alert.detail}
        </div>
      )}

      <main className={styles.grid}>
        <Panel
          title="Pipeline"
          tone={qTone}
          note={data?.queue.in_flight ? `${data.queue.in_flight} in flight` : "idle"}
        >
          <StatGrid>
            <Stat
              label="Backlog"
              value={count(data?.jobs.queued)}
              tone={qTone}
              caption={
                data?.jobs.oldest_queued_age_s != null
                  ? `oldest unclaimed ${age(data.jobs.oldest_queued_age_s)}`
                  : "nothing waiting"
              }
            />
            <Stat
              label="Claimed"
              value={count(data?.jobs.running)}
              caption={`${count(data?.queue.depth)} queued on this replica`}
            />
            <Stat label="In flight" value={count(data?.queue.in_flight)} caption="classifying here" />
            <Stat
              label="Latency p50"
              value={seconds(data?.latency.p50_s)}
              caption="enqueue → finish"
            />
            <Stat label="Latency p95" value={seconds(data?.latency.p95_s)} caption="worst 5%" />
            <Stat
              label="Failure rate"
              value={percent(data?.latency.failure_rate)}
              tone={fTone}
              caption={`${count(data?.latency.failed_in_window)} of ${count(data?.latency.finished_in_window)} in window`}
            />
            <Stat
              label="Classify cost"
              value={seconds(data?.queue.avg_classify_s)}
              caption="mean, this process"
            />
          </StatGrid>
        </Panel>

        <Panel title="Ingest" note={`rate over the last ${WINDOWS.find((w) => w.value === windowS)?.label}`}>
          <StatGrid>
            <Stat
              label="Frames / min"
              value={data ? data.ingest.frames_per_min_window.toFixed(1) : "—"}
              tone={data && data.ingest.frames_last_1m > 0 ? "good" : "idle"}
              caption={`${count(data?.ingest.frames_in_window)} in window`}
            />
            <Stat
              label="Last frame"
              value={idleS == null ? "—" : age(idleS)}
              caption={idleS == null ? "no frames ingested" : "ago"}
            />
            <Stat label="Last hour" value={count(data?.ingest.frames_last_1h)} caption="frames" />
          </StatGrid>
          <p className={styles.sparkNote}>frames per minute · sampled since this page opened</p>
          <Sparkline points={spark} unit="f/min" />
        </Panel>

        <Panel
          title="Fleet"
          tone={data?.fleet.missions_active ? "good" : "idle"}
          note={
            data
              ? `${count(data.fleet.drones_active)} of ${count(data.fleet.drones_total)} drones seen in window`
              : undefined
          }
        >
          {data ? <FleetTable fleet={data.fleet} /> : <p className={styles.loading}>loading…</p>}
        </Panel>

        <Panel
          title="Coverage"
          note={data ? `freshness window ${age(data.config.occupancy_window_s)}` : undefined}
        >
          {data ? <CoverageBar c={data.coverage} /> : <p className={styles.loading}>loading…</p>}
        </Panel>

        <Panel
          title="Model accuracy"
          tone={accuracyTone(data?.model.state_accuracy ?? null)}
          note={data?.model.bays_scored ? "vs ground truth" : "no ground truth"}
        >
          {data && data.model.bays_scored === 0 ? (
            // Production has no labels; saying so beats rendering a 0% score.
            <p className={styles.loading}>
              No labelled bays. Accuracy appears once a survey with ground truth
              (a sim or eval run) has been ingested.
            </p>
          ) : (
            <StatGrid>
              <Stat
                label="State accuracy"
                value={percent(data?.model.state_accuracy)}
                tone={accuracyTone(data?.model.state_accuracy ?? null)}
                caption={`${count(data?.model.bays_correct)} of ${count(data?.model.bays_scored)} bays, after voting`}
              />
              <Stat
                label="Per-view accuracy"
                value={percent(data?.model.view_accuracy)}
                tone={accuracyTone(data?.model.view_accuracy ?? null)}
                caption={`${count(data?.model.views_correct)} of ${count(data?.model.views_scored)} single looks`}
              />
            </StatGrid>
          )}
          <p className={styles.sparkNote}>
            state accuracy is the product-level number: one verdict per bay after the
            majority vote. Per-view is the raw classifier, before voting cancels a bad look.
          </p>
        </Panel>

        <Panel title="This process" note="resets on restart">
          <StatGrid>
            <Stat label="Classified" value={count(data?.queue.processed_lifetime)} caption="frames" />
            <Stat
              label="Failed"
              value={count(data?.queue.failed_lifetime)}
              tone={data && data.queue.failed_lifetime > 0 ? "warn" : "idle"}
              caption="unrecoverable"
            />
            <Stat
              label="Reclaimed"
              value={count(data?.queue.reclaimed_lifetime)}
              tone={data ? leaseTone(data.jobs.expired_leases) : "idle"}
              caption={
                data?.jobs.expired_leases
                  ? `${count(data.jobs.expired_leases)} lease(s) lapsed now`
                  : "from a lapsed lease"
              }
            />
            <Stat
              label="Deltas produced"
              value={count(data?.queue.deltas_pushed_lifetime)}
              caption="by frames classified here"
            />
            <Stat
              label="Jobs on record"
              value={count(
                data
                  ? data.jobs.processed + data.jobs.failed + data.jobs.queued + data.jobs.running
                  : null,
              )}
              caption={data ? `within ${age(data.config.frame_retention_s)} retention` : undefined}
            />
          </StatGrid>
        </Panel>
      </main>

      <footer className={styles.foot}>
        GET /api/v1/metrics · polled every 3 s · Prometheus exposition at /metrics
      </footer>
    </div>
  );
}

function buildAlert(
  qTone: Tone,
  fTone: Tone,
  oldestQueuedS: number | null,
  error: string | null,
): { tone: Tone; title: string; detail: string } | null {
  if (error) {
    return {
      tone: "critical",
      title: "Server unreachable.",
      detail: `${error} — the figures below are the last good reading.`,
    };
  }
  if (oldestQueuedS != null && oldestQueuedS >= STALL_WARN_S) {
    return {
      tone: oldestQueuedS >= STALL_CRIT_S ? "critical" : "warn",
      title: "Classify pipeline is stalling.",
      detail: `The oldest job has been queued for ${age(oldestQueuedS)}. Check the classify threads and the object store.`,
    };
  }
  if (fTone === "warn" || fTone === "critical") {
    return {
      tone: fTone,
      title: "Frames are failing to classify.",
      detail: "Usually a missing image in the object store — check the server log for `classify failed`.",
    };
  }
  if (qTone === "warn") {
    return { tone: "warn", title: "Queue is backing up.", detail: "Depth is high but jobs are still moving." };
  }
  return null;
}
