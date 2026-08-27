import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { loadEnv } from "./env.js";
import { closePool, getPool } from "./pool.js";

loadEnv();

const here = dirname(fileURLToPath(import.meta.url));

// web/packages/db/src -> ../../../../data
const DEFAULT_GEOJSON = resolve(here, "..", "..", "..", "..", "data", "curb_runs.geojson");

interface Feature {
  properties: Record<string, unknown>;
  geometry: { type: string; coordinates: unknown };
}

/**
 * Load data/curb_runs.geojson (from tools/make_runs.py) into `curb_run`.
 *
 * Runs are reference data derived from the bays, so this is a sibling of
 * `seed.ts` rather than part of it: re-deriving the runs is a deliberate act
 * (the chaining constants can change) and it must not be a side effect of
 * re-seeding bays. Idempotent -- ON CONFLICT re-upserts.
 *
 * UNVERIFIED chains (fewer than 4 bays: courtyards, driveway stubs) are loaded
 * too, with `verified=false`. Dropping them would erase the difference between
 * "we do not know" and "there is nothing there", which is the same distinction
 * the rest of this pipeline works to preserve.
 */
async function main(): Promise<void> {
  const path = process.env.RUNS_GEOJSON
    ? resolve(process.cwd(), process.env.RUNS_GEOJSON)
    : DEFAULT_GEOJSON;
  console.log(`seeding curb runs from ${path}`);

  const fc = JSON.parse(readFileSync(path, "utf8")) as { features: Feature[] };
  const pool = getPool();
  const client = await pool.connect();

  let inserted = 0;
  let skipped = 0;
  let verified = 0;
  try {
    await client.query("BEGIN");
    for (const f of fc.features) {
      const p = f.properties;
      const runId = typeof p.run_id === "string" ? p.run_id : null;
      if (!runId || f.geometry?.type !== "LineString") {
        skipped++;
        continue;
      }
      const isVerified = p.verified !== false;
      if (isVerified) verified++;
      await client.query(
        `INSERT INTO curb_run
           (run_id, street, side, zona, park_txt, capacity, pitch_m, length_m,
            verified, bay_ids, geom)
         VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,
                 ST_SetSRID(ST_GeomFromGeoJSON($11), 4326))
         ON CONFLICT (run_id) DO UPDATE SET
           street = EXCLUDED.street, side = EXCLUDED.side, zona = EXCLUDED.zona,
           park_txt = EXCLUDED.park_txt, capacity = EXCLUDED.capacity,
           pitch_m = EXCLUDED.pitch_m, length_m = EXCLUDED.length_m,
           verified = EXCLUDED.verified, bay_ids = EXCLUDED.bay_ids,
           geom = EXCLUDED.geom`,
        [
          runId,
          nullableStr(p.street),
          nullableStr(p.side),
          nullableStr(p.zona),
          nullableStr(p.park_txt),
          typeof p.capacity === "number" ? p.capacity : 0,
          typeof p.pitch_m === "number" ? p.pitch_m : null,
          typeof p.length_m === "number" ? p.length_m : null,
          isVerified,
          Array.isArray(p.bay_ids) ? p.bay_ids.map(String) : [],
          JSON.stringify(f.geometry),
        ],
      );
      inserted++;
    }
    await client.query("COMMIT");
  } catch (err) {
    await client.query("ROLLBACK");
    throw err;
  } finally {
    client.release();
  }
  console.log(
    `seeded ${inserted} curb runs (${verified} verified, ${skipped} skipped)`,
  );
}

function nullableStr(v: unknown): string | null {
  if (typeof v !== "string") return null;
  const t = v.trim();
  return t === "" || t === "-" ? null : t;
}

main()
  .catch((err) => {
    console.error(err);
    process.exitCode = 1;
  })
  .finally(() => closePool());
