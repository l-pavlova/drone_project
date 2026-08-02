/** Mirrors the payload of GET /api/v1/metrics (parkdrone_vision/api/metrics.py).
 *  Nullables are real: a figure with no samples in the window is reported as
 *  null rather than 0, because "nothing happened" and "zero seconds" are
 *  different answers and the UI renders them differently. */

export interface ProcessMetrics {
  uptime_s: number;
  classify_threads: number;
  workers_started: number;
  hub_clients: number;
}

export interface QueueMetrics {
  depth: number;
  in_flight: number;
  processed_lifetime: number;
  failed_lifetime: number;
  recovered_on_start: number;
  deltas_pushed_lifetime: number;
  avg_classify_s: number | null;
}

export interface JobMetrics {
  queued: number;
  processed: number;
  failed: number;
  oldest_queued_age_s: number | null;
}

export interface IngestMetrics {
  frames_last_1m: number;
  frames_last_1h: number;
  frames_in_window: number;
  frames_per_min_window: number;
  last_frame_at: string | null;
}

export interface LatencyMetrics {
  finished_in_window: number;
  avg_s: number | null;
  p50_s: number | null;
  p95_s: number | null;
  failed_in_window: number;
  failure_rate: number | null;
}

export interface MissionRow {
  mission_id: string;
  drone_id: string;
  survey_area: string;
  started_at: string;
  frames_done: number;
  frames_expected: number | null;
  since_last_frame_s: number | null;
}

export interface FleetMetrics {
  drones_total: number;
  drones_active: number;
  missions_active: number;
  missions: MissionRow[];
}

export interface CoverageMetrics {
  bays_total: number;
  bays_occupied: number;
  bays_free: number;
  bays_unknown: number;
}

/** Accuracy is null wherever there is no ground truth — production has none.
 *  "unknown" and "zero" must stay distinguishable all the way to the tile. */
export interface ModelMetrics {
  views_scored: number;
  views_correct: number;
  view_accuracy: number | null;
  bays_scored: number;
  bays_correct: number;
  state_accuracy: number | null;
}

export interface Metrics {
  generated_at: number;
  window_s: number;
  process: ProcessMetrics;
  queue: QueueMetrics;
  jobs: JobMetrics;
  ingest: IngestMetrics;
  latency: LatencyMetrics;
  fleet: FleetMetrics;
  coverage: CoverageMetrics;
  model: ModelMetrics;
  config: { occupancy_window_s: number; frame_retention_s: number };
}

/** Severity of a reading. Status colours are reserved for these four and never
 *  reused as chart series (they always ship with a label, never colour alone). */
export type Tone = "good" | "idle" | "warn" | "critical";
