import { Router } from "express";
import multer from "multer";
import { v4 as uuid } from "uuid";
import { getPool } from "@parkdrone/db";
import {
  ingestFrameMetaSchema,
  missionStartSchema,
} from "@parkdrone/contracts";
import { requireDrone } from "../auth.js";
import { enqueueFrameJob } from "../redis.js";
import { putFrame } from "../s3.js";

const upload = multer({
  storage: multer.memoryStorage(),
  limits: { fileSize: 8 * 1024 * 1024 }, // 8 MB/frame cap
});

export const ingestRouter: Router = Router();
ingestRouter.use(requireDrone);

/** Start a survey run. */
ingestRouter.post("/mission/start", async (req, res) => {
  const parsed = missionStartSchema.safeParse({
    ...req.body,
    drone_id: req.drone!.droneId,
  });
  if (!parsed.success) {
    return res.status(400).json({ error: parsed.error.flatten() });
  }
  const missionId = uuid();
  await getPool().query(
    `INSERT INTO mission (mission_id, drone_id, world, area, frames_expected)
     VALUES ($1, $2, $3, $4, $5)`,
    [
      missionId,
      parsed.data.drone_id,
      parsed.data.world,
      parsed.data.area ?? null,
      parsed.data.frames_expected ?? null,
    ],
  );
  res.status(201).json({ mission_id: missionId });
});

/** End a survey run. */
ingestRouter.post("/mission/:id/end", async (req, res) => {
  await getPool().query(
    "UPDATE mission SET ended_at = now() WHERE mission_id = $1 AND drone_id = $2",
    [req.params.id, req.drone!.droneId],
  );
  res.json({ ok: true });
});

/**
 * Ingest one frame: multipart with `frame` (PNG) + `meta` (JSON). Stores the
 * image, records the frame, enqueues a vision job. 202 on accept. Idempotent on
 * (drone_id, world, i): a re-send returns the existing frame_id without requeue.
 */
ingestRouter.post("/frame", upload.single("frame"), async (req, res) => {
  if (!req.file) return res.status(400).json({ error: "missing frame file" });

  let metaRaw: unknown;
  try {
    metaRaw = JSON.parse(req.body.meta);
  } catch {
    return res.status(400).json({ error: "meta must be JSON" });
  }
  const parsed = ingestFrameMetaSchema.safeParse(metaRaw);
  if (!parsed.success) {
    return res.status(400).json({ error: parsed.error.flatten() });
  }
  const meta = parsed.data;
  if (meta.drone_id !== req.drone!.droneId) {
    return res.status(403).json({ error: "drone_id mismatch" });
  }
  const { pose, world } = meta;

  // idempotent frame insert
  const frameId = uuid();
  const key = `${world}/${meta.drone_id}/frame_${String(pose.i).padStart(3, "0")}.png`;
  const imageUri = await putFrame(key, req.file.buffer);

  const { rows } = await getPool().query(
    `INSERT INTO frame
       (frame_id, drone_id, mission_id, world, i, x, y, alt, yaw, roll, pitch, image_uri)
     VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,$12)
     ON CONFLICT (drone_id, world, i) DO NOTHING
     RETURNING frame_id`,
    [
      frameId,
      meta.drone_id,
      meta.mission_id ?? null,
      world,
      pose.i,
      pose.x,
      pose.y,
      pose.alt,
      pose.yaw,
      pose.roll,
      pose.pitch,
      imageUri,
    ],
  );

  if (!rows.length) {
    const existing = await getPool().query(
      "SELECT frame_id FROM frame WHERE drone_id = $1 AND world = $2 AND i = $3",
      [meta.drone_id, world, pose.i],
    );
    return res.status(200).json({ frame_id: existing.rows[0]?.frame_id, duplicate: true });
  }

  if (meta.mission_id) {
    await getPool().query(
      "UPDATE mission SET frames_done = frames_done + 1 WHERE mission_id = $1",
      [meta.mission_id],
    );
  }

  await enqueueFrameJob({
    frame_id: frameId,
    world,
    frame_idx: pose.i,
    pose,
    image_uri: imageUri,
  });

  res.status(202).json({ frame_id: frameId });
});
