import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocket } from "ws";

/**
 * End-to-end harness: replay a sim world's frames through the LIVE stack
 * (ingest API -> Redis queue -> Python worker -> Redis pub/sub -> WebSocket) and
 * assert the pushed deltas + final /bays state match the offline result. This is
 * the plan's "ingest -> push loop" verification, run without a real drone.
 *
 *   API_KEY=<key> tsx src/scripts/replay-ingest.ts [world] [apiBase]
 */
const here = dirname(fileURLToPath(import.meta.url));
const repoRoot = join(here, "..", "..", "..", "..", ".."); // scripts->src->api->apps->web->root

const world = process.argv[2] ?? "fmi_block";
const apiBase = process.argv[3] ?? "http://localhost:4000";
const apiKey = process.env.API_KEY;
if (!apiKey) throw new Error("set API_KEY (from register-drone)");

const droneId = process.env.DRONE_ID ?? "drone-1";
const outDir = join(repoRoot, "sim", "output", world);

interface Pose {
  i: number;
  x: number;
  y: number;
  alt: number;
  yaw: number;
  roll: number;
  pitch: number;
  wp: [number, number];
}

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

async function main(): Promise<void> {
  const poses: Pose[] = JSON.parse(readFileSync(join(outDir, "poses.json"), "utf8"));
  const expected = JSON.parse(
    readFileSync(join(outDir, "occupancy_results.json"), "utf8"),
  ).results as Record<string, { pred: boolean }>;

  // 1) collect pushed deltas over the WebSocket
  const received = new Map<string, boolean>();
  const ws = new WebSocket(`${apiBase.replace("http", "ws")}/ws/occupancy`);
  await new Promise<void>((resolve, reject) => {
    ws.once("open", () => resolve());
    ws.once("error", reject);
  });
  ws.on("message", (buf) => {
    const msg = JSON.parse(buf.toString());
    if (msg.type === "bay_delta") received.set(msg.bay_id, msg.occupied);
  });
  console.log("ws connected");

  // 2) start a mission
  const mission = await postJson(`${apiBase}/api/v1/ingest/mission/start`, {
    world,
    area: "verification",
    frames_expected: poses.length,
  });
  console.log("mission", mission.mission_id);

  // 3) stream every frame through the ingest endpoint
  for (const pose of poses) {
    const png = readFileSync(join(outDir, `frame_${String(pose.i).padStart(3, "0")}.png`));
    const form = new FormData();
    form.append("frame", new Blob([png], { type: "image/png" }), `frame_${pose.i}.png`);
    form.append(
      "meta",
      JSON.stringify({ drone_id: droneId, world, mission_id: mission.mission_id, pose }),
    );
    const res = await fetch(`${apiBase}/api/v1/ingest/frame`, {
      method: "POST",
      headers: { "x-api-key": apiKey! },
      body: form,
    });
    if (res.status !== 202) console.warn(`frame ${pose.i}: HTTP ${res.status}`);
  }
  console.log(`posted ${poses.length} frames`);

  // 4) wait for the worker to drain + deltas to settle. Break only once we've
  //    actually received deltas AND the count has been stable for 2s (avoids
  //    bailing during the initial gap before the first delta arrives).
  let prev = -1;
  let stable = 0;
  for (let t = 0; t < 40; t++) {
    await sleep(1000);
    if (received.size > 0 && received.size === prev) {
      if (++stable >= 2) break;
    } else {
      stable = 0;
    }
    prev = received.size;
  }
  await postJson(`${apiBase}/api/v1/ingest/mission/${mission.mission_id}/end`, {});
  ws.close();

  // 5) assert final /bays state matches the offline predictions
  const fc = (await (await fetch(`${apiBase}/api/v1/bays`)).json()) as {
    features: Array<{ properties: { bay_id: string; occupied: boolean | null } }>;
  };
  const stateById = new Map<string, boolean | null>();
  for (const f of fc.features) {
    stateById.set(String(f.properties.bay_id), f.properties.occupied);
  }
  let match = 0;
  let mismatch = 0;
  const bad: string[] = [];
  for (const [bayId, r] of Object.entries(expected)) {
    const got = stateById.get(bayId);
    if (got === r.pred) match++;
    else {
      mismatch++;
      bad.push(`${bayId}: offline=${r.pred} live=${got}`);
    }
  }

  console.log(`\nWebSocket deltas received: ${received.size}`);
  console.log(`final /bays vs occupancy_results: match=${match} mismatch=${mismatch}`);
  if (bad.length) console.log("  " + bad.slice(0, 20).join("\n  "));
  process.exit(mismatch ? 1 : 0);
}

async function postJson(url: string, body: unknown): Promise<any> {
  const res = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json", "x-api-key": apiKey! },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`POST ${url} -> ${res.status}: ${await res.text()}`);
  return res.json();
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
