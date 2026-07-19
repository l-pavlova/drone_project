import type { Server } from "node:http";
import { WebSocketServer, type WebSocket } from "ws";
import { frameScoredSchema, type BayDelta } from "@parkdrone/contracts";
import { config } from "./config.js";
import { makeSubscriber } from "./redis.js";

interface Client {
  socket: WebSocket;
  bbox?: [number, number, number, number]; // minLon,minLat,maxLon,maxLat
}

// Small in-memory replay buffer so a reconnecting client can catch up with
// `?since=<cursor>`. Bounded; for durable replay use a Redis stream (see plan).
const REPLAY_MAX = 500;
const replay: Array<{ cursor: number; delta: BayDelta }> = [];
let cursorSeq = 0;

/**
 * Attach the occupancy WebSocket at /ws/occupancy and bridge Redis pub/sub
 * (worker deltas) to all connected clients. This fan-out works across
 * horizontally-scaled API instances because each subscribes to the same channel.
 */
export function attachOccupancyWs(server: Server): void {
  const wss = new WebSocketServer({ server, path: "/ws/occupancy" });

  wss.on("connection", (socket, req) => {
    const url = new URL(req.url ?? "", "http://localhost");
    const client: Client = { socket };

    const bboxParam = url.searchParams.get("bbox");
    if (bboxParam) {
      const p = bboxParam.split(",").map(Number);
      if (p.length === 4 && p.every(Number.isFinite)) {
        client.bbox = p as [number, number, number, number];
      }
    }

    // replay missed deltas
    const since = Number(url.searchParams.get("since"));
    if (Number.isFinite(since) && since > 0) {
      for (const item of replay) {
        if (item.cursor > since) send(client, item.delta);
      }
    }
    send(client, { type: "snapshot_cursor", cursor: String(cursorSeq) });

    socket.on("pong", () => {
      /* liveness tracked by ws internal; no-op */
    });
  });

  // heartbeat
  const ping = setInterval(() => {
    for (const c of wss.clients) {
      if (c.readyState === c.OPEN) c.ping();
    }
  }, 30_000);
  wss.on("close", () => clearInterval(ping));

  // Redis subscription: worker publishes FrameScored payloads.
  const sub = makeSubscriber();
  sub.subscribe(config.deltasChannel).catch((e) => {
    console.error("failed to subscribe to deltas channel", e);
  });
  sub.on("message", (_channel, message) => {
    const parsed = frameScoredSchema.safeParse(safeJson(message));
    if (!parsed.success) return;
    for (const delta of parsed.data.changed) {
      const cursor = ++cursorSeq;
      replay.push({ cursor, delta });
      if (replay.length > REPLAY_MAX) replay.shift();
      for (const c of wss.clients) {
        broadcast(c, delta);
      }
    }
  });

  function broadcast(socket: WebSocket, delta: BayDelta): void {
    if (socket.readyState === socket.OPEN) {
      socket.send(JSON.stringify(delta));
    }
  }
}

function send(client: Client, msg: unknown): void {
  if (client.socket.readyState === client.socket.OPEN) {
    client.socket.send(JSON.stringify(msg));
  }
}

function safeJson(s: string): unknown {
  try {
    return JSON.parse(s);
  } catch {
    return null;
  }
}
