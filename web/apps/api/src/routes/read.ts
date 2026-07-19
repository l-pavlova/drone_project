import { Router } from "express";
import { bayRepo, type Bbox } from "@parkdrone/db";

export const readRouter: Router = Router();

/** Parse "minLon,minLat,maxLon,maxLat" -> Bbox (or undefined). */
function parseBbox(raw: unknown): Bbox | undefined {
  if (typeof raw !== "string") return undefined;
  const parts = raw.split(",").map(Number);
  if (parts.length !== 4 || parts.some((n) => !Number.isFinite(n))) {
    throw new Error("bbox must be minLon,minLat,maxLon,maxLat");
  }
  const [minLon, minLat, maxLon, maxLat] = parts as [number, number, number, number];
  return { minLon, minLat, maxLon, maxLat };
}

/** Bays + current occupancy as a GeoJSON FeatureCollection. */
readRouter.get("/bays", async (req, res) => {
  let bbox: Bbox | undefined;
  try {
    bbox = parseBbox(req.query.bbox);
  } catch (e) {
    return res.status(400).json({ error: (e as Error).message });
  }
  const zona = typeof req.query.zona === "string" ? req.query.zona : undefined;
  const fc = await bayRepo.featureCollection({ bbox, zona });
  res.json(fc);
});

/** Free/occupied/unknown counts per zone. */
readRouter.get("/summary", async (_req, res) => {
  res.json({ zones: await bayRepo.summary() });
});

/** One bay + current state + recent observations. */
readRouter.get("/bays/:id", async (req, res) => {
  const detail = await bayRepo.detail(req.params.id);
  if (!detail) return res.status(404).json({ error: "bay not found" });
  res.json(detail);
});
