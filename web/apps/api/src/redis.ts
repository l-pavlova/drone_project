import { Redis } from "ioredis";
import { config } from "./config.js";

/** Command connection (LPUSH jobs, one-off ops). */
export const redis = new Redis(config.redisUrl, { maxRetriesPerRequest: null });

/** A dedicated connection for SUBSCRIBE (ioredis pub/sub needs its own socket). */
export function makeSubscriber(): Redis {
  return new Redis(config.redisUrl, { maxRetriesPerRequest: null });
}

/** Enqueue a frame job for the Python vision worker (BRPOP consumer). */
export async function enqueueFrameJob(job: unknown): Promise<void> {
  await redis.lpush(config.jobsQueue, JSON.stringify(job));
}

export interface DeltaOut {
  bay_id: string;
  occupied: boolean;
  confidence: number;
  updated_at: string;
}

/** Publish occupancy deltas on the same channel the worker uses, so the WS
 *  fan-out forwards them to dashboards. Used by the vision worker path and the
 *  dev/test endpoints alike. */
export async function publishDeltas(
  world: string,
  frameIdx: number,
  deltas: DeltaOut[],
): Promise<void> {
  await redis.publish(
    config.deltasChannel,
    JSON.stringify({
      world,
      frame_idx: frameIdx,
      changed: deltas.map((d) => ({ type: "bay_delta", ...d })),
    }),
  );
}
