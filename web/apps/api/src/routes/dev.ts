import { Router } from "express";
import { bayRepo, getPool } from "@parkdrone/db";
import { publishDeltas } from "../redis.js";

/**
 * Dev/test endpoints for driving the dashboard by hand — occupy or free a bay
 * and watch the map update live. These bypass the drone/vision pipeline: they
 * write bay_state as a MANUAL override and publish a delta on the same Redis
 * channel the worker uses, so the WebSocket pushes it to every client.
 *
 * Not for production — mounted only when ENABLE_DEV_ROUTES !== "false".
 *
 *   POST /api/v1/dev/occupy        # occupy the bay in front of FMI
 *   POST /api/v1/dev/free          # free it again
 *   POST /api/v1/dev/occupy?bay_id=17571   # target a specific bay
 */
export const devRouter: Router = Router();

// FMI block origin (matches the ENU ORIGIN used across the project).
const FMI = { lon: 23.3298956, lat: 42.6747105 };

/** Resolve the target bay: an explicit ?bay_id, else the bay nearest to FMI. */
async function resolveBay(bayId?: unknown): Promise<string | null> {
  if (typeof bayId === "string" && bayId.trim()) {
    const { rowCount } = await getPool().query("SELECT 1 FROM bay WHERE bay_id = $1", [
      bayId.trim(),
    ]);
    return rowCount ? bayId.trim() : null;
  }
  const { rows } = await getPool().query(
    `SELECT bay_id, street FROM bay
      ORDER BY centroid <-> ST_SetSRID(ST_MakePoint($1, $2), 4326)
      LIMIT 1`,
    [FMI.lon, FMI.lat],
  );
  return rows[0]?.bay_id ?? null;
}

async function setOccupancy(bayId: string, occupied: boolean) {
  await bayRepo.upsertState({
    bayId,
    occupied,
    confidence: 1,
    lastFrame: null,
    source: "manual",
  });
  const updatedAt = new Date().toISOString();
  await publishDeltas("manual", 0, [
    { bay_id: bayId, occupied, confidence: 1, updated_at: updatedAt },
  ]);
  return updatedAt;
}

function handler(occupied: boolean) {
  return async (req: import("express").Request, res: import("express").Response) => {
    const bayId = await resolveBay(req.query.bay_id);
    if (!bayId) return res.status(404).json({ error: "no matching bay" });
    const updatedAt = await setOccupancy(bayId, occupied);
    res.json({ bay_id: bayId, occupied, updated_at: updatedAt });
  };
}

devRouter.post("/occupy", handler(true));
devRouter.post("/free", handler(false));
