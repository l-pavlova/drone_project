import { z } from "zod";

export * from "./geo.js";

/**
 * Bay ids are integers in `data/block_bays.geojson` but strings in
 * `ground_truth.json` / `occupancy_results.json`. The whole web tier
 * canonicalizes to STRING. Use {@link toBayId} at every boundary.
 */
export type BayId = string;
export function toBayId(raw: string | number): BayId {
  return String(raw).trim();
}

// ---------------------------------------------------------------------------
// Pose — one captured frame's drone state. Matches a record in
// sim/output/<world>/poses.json (parkdrone.py). `cam_*` are optional (only
// present when the Webots proto exposes those gimbal sensors).
// ---------------------------------------------------------------------------
export const poseSchema = z.object({
  i: z.number().int().nonnegative(),
  x: z.number(),
  y: z.number(),
  alt: z.number(),
  yaw: z.number(),
  roll: z.number(),
  pitch: z.number(),
  wp: z.tuple([z.number(), z.number()]),
  cam_pitch: z.number().optional(),
  cam_roll: z.number().optional(),
  cam_yaw: z.number().optional(),
});
export type Pose = z.infer<typeof poseSchema>;

// ---------------------------------------------------------------------------
// Ingest — metadata a drone sends alongside a frame image.
// ---------------------------------------------------------------------------
export const ingestFrameMetaSchema = z.object({
  drone_id: z.string().min(1),
  world: z.string().min(1),
  mission_id: z.string().min(1).optional(),
  pose: poseSchema,
});
export type IngestFrameMeta = z.infer<typeof ingestFrameMetaSchema>;

export const missionStartSchema = z.object({
  drone_id: z.string().min(1),
  world: z.string().min(1),
  area: z.string().optional(),
  frames_expected: z.number().int().nonnegative().optional(),
});
export type MissionStart = z.infer<typeof missionStartSchema>;

// ---------------------------------------------------------------------------
// Bay — static geometry + attributes from block_bays.geojson.
// `public` is derived from vid_txt_20 === "Зона" (see generate_world.py).
// ---------------------------------------------------------------------------
export const parkOrientation = z.enum(["longitudinal", "perpendicular", "unknown"]);
export type ParkOrientation = z.infer<typeof parkOrientation>;

export interface BayProperties {
  bay_id: BayId;
  zona: string | null;
  street: string | null;
  park_txt: string | null;
  bearing_deg: number | null;
  public: boolean;
}

// ---------------------------------------------------------------------------
// Occupancy state + the delta pushed over WebSocket.
// ---------------------------------------------------------------------------
export const occupancySource = z.enum(["vision", "manual"]);
export type OccupancySource = z.infer<typeof occupancySource>;

export interface BayState {
  bay_id: BayId;
  occupied: boolean;
  confidence: number;
  last_frame: number | null;
  updated_at: string; // ISO-8601
  source: OccupancySource;
}

export const bayDeltaSchema = z.object({
  type: z.literal("bay_delta"),
  bay_id: z.string(),
  occupied: z.boolean(),
  confidence: z.number(),
  updated_at: z.string(),
});
export type BayDelta = z.infer<typeof bayDeltaSchema>;

/** WebSocket envelope: server -> client messages on /ws/occupancy. */
export const wsServerMessageSchema = z.discriminatedUnion("type", [
  bayDeltaSchema,
  z.object({ type: z.literal("ping"), t: z.number() }),
  z.object({ type: z.literal("snapshot_cursor"), cursor: z.string() }),
]);
export type WsServerMessage = z.infer<typeof wsServerMessageSchema>;

// ---------------------------------------------------------------------------
// Observation — an append-only per-frame classification record. Superset of
// one per-bay entry in occupancy_results.json (`gt` optional: only in eval).
// ---------------------------------------------------------------------------
export const observationSchema = z.object({
  bay_id: z.string(),
  occupied: z.boolean(),
  frame_idx: z.number().int(),
  world: z.string(),
  votes_occupied: z.number().int().nonnegative(),
  views: z.number().int().nonnegative(),
  vis: z.number(),
  center_off_px: z.number().optional(),
  core_paint_frac: z.number().optional(),
  core_dark_frac: z.number().optional(),
  core_chroma: z.number().optional(),
  core_brightness: z.number().optional(),
  core_std: z.number().optional(),
  gt: z.boolean().nullable().optional(),
});
export type Observation = z.infer<typeof observationSchema>;

/**
 * Worker -> API delta payload published on Redis after a frame is scored.
 * Carries the changed bay states plus the frame that produced them.
 */
export const frameScoredSchema = z.object({
  frame_id: z.string().nullable().optional(),
  world: z.string(),
  frame_idx: z.number().int(),
  // full per-bay observations are optional on the pub/sub hot path; the WS
  // fan-out only needs `changed`. The worker omits them to keep messages small.
  observations: z.array(observationSchema).optional().default([]),
  changed: z.array(bayDeltaSchema),
});
export type FrameScored = z.infer<typeof frameScoredSchema>;
