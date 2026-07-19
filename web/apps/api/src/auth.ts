import { createHash } from "node:crypto";
import type { NextFunction, Request, Response } from "express";
import { getPool } from "@parkdrone/db";

/** SHA-256 of a drone API key (matches drone.api_key_hash at rest). */
export function hashApiKey(key: string): string {
  return createHash("sha256").update(key).digest("hex");
}

export interface DroneAuth {
  droneId: string;
}

declare global {
  // eslint-disable-next-line @typescript-eslint/no-namespace
  namespace Express {
    interface Request {
      drone?: DroneAuth;
    }
  }
}

/**
 * Authenticate a drone by its API key (header `x-api-key`). Looks up the hash
 * in the drone table, sets req.drone, and bumps last_seen.
 */
export async function requireDrone(
  req: Request,
  res: Response,
  next: NextFunction,
): Promise<void> {
  const key = req.header("x-api-key");
  if (!key) {
    res.status(401).json({ error: "missing x-api-key" });
    return;
  }
  const { rows } = await getPool().query(
    "SELECT drone_id FROM drone WHERE api_key_hash = $1",
    [hashApiKey(key)],
  );
  if (!rows.length) {
    res.status(403).json({ error: "invalid api key" });
    return;
  }
  req.drone = { droneId: rows[0].drone_id };
  await getPool().query("UPDATE drone SET last_seen = now() WHERE drone_id = $1", [
    rows[0].drone_id,
  ]);
  next();
}
