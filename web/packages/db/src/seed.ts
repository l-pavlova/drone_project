import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { toBayId } from "@parkdrone/contracts";
import { loadEnv } from "./env.js";
import { closePool, getPool } from "./pool.js";

loadEnv();

const here = dirname(fileURLToPath(import.meta.url));

// Default to the repo's canonical bay geometry: web/packages/db/src -> ../../../../data
const DEFAULT_GEOJSON = resolve(here, "..", "..", "..", "..", "data", "block_bays.geojson");

interface Feature {
  properties: Record<string, unknown>;
  geometry: { type: string; coordinates: unknown };
}

/**
 * Load block_bays.geojson into the `bay` table. Geometry goes in via
 * ST_GeomFromGeoJSON (WGS84); `public` mirrors generate_world.py
 * (vid_txt_20 === "Зона"); bay ids are canonicalized to strings.
 * Idempotent: ON CONFLICT re-upserts geometry/attributes.
 */
async function main(): Promise<void> {
  const path = process.env.BAYS_GEOJSON
    ? resolve(process.cwd(), process.env.BAYS_GEOJSON)
    : DEFAULT_GEOJSON;
  console.log(`seeding bays from ${path}`);

  const fc = JSON.parse(readFileSync(path, "utf8")) as { features: Feature[] };
  const pool = getPool();
  const client = await pool.connect();

  let inserted = 0;
  let skipped = 0;
  try {
    await client.query("BEGIN");
    for (const f of fc.features) {
      const p = f.properties;
      const bayId = p.id == null ? null : toBayId(p.id as string | number);
      if (!bayId || f.geometry?.type !== "Polygon") {
        skipped++;
        continue;
      }
      const isPublic = p.vid_txt_20 === "Зона";
      const bearing =
        typeof p.bearing_deg === "number" ? p.bearing_deg : null;

      await client.query(
        `INSERT INTO bay (bay_id, zona, street, park_txt, bearing_deg, public, geom, centroid)
         VALUES ($1, $2, $3, $4, $5, $6,
                 ST_SetSRID(ST_GeomFromGeoJSON($7), 4326),
                 ST_Centroid(ST_SetSRID(ST_GeomFromGeoJSON($7), 4326)))
         ON CONFLICT (bay_id) DO UPDATE SET
           zona = EXCLUDED.zona,
           street = EXCLUDED.street,
           park_txt = EXCLUDED.park_txt,
           bearing_deg = EXCLUDED.bearing_deg,
           public = EXCLUDED.public,
           geom = EXCLUDED.geom,
           centroid = EXCLUDED.centroid`,
        [
          bayId,
          nullableStr(p.zona),
          nullableStr(p.mestopoloz),
          nullableStr(p.park_txt),
          bearing,
          isPublic,
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
  console.log(`seeded ${inserted} bays (${skipped} skipped)`);
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
