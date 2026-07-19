import { randomBytes } from "node:crypto";
import "../config.js"; // loads the monorepo-root .env (sets DATABASE_URL)
import { closePool, getPool } from "@parkdrone/db";
import { hashApiKey } from "../auth.js";

/**
 * Register (or re-key) a drone and print a fresh API key.
 *   tsx src/scripts/register-drone.ts <drone_id> [name]
 * The plaintext key is shown ONCE; only its hash is stored.
 */
async function main(): Promise<void> {
  const droneId = process.argv[2];
  const name = process.argv[3] ?? droneId;
  if (!droneId) {
    console.error("usage: register-drone <drone_id> [name]");
    process.exit(1);
  }
  const key = randomBytes(24).toString("hex");
  await getPool().query(
    `INSERT INTO drone (drone_id, name, api_key_hash)
     VALUES ($1, $2, $3)
     ON CONFLICT (drone_id) DO UPDATE SET name = EXCLUDED.name, api_key_hash = EXCLUDED.api_key_hash`,
    [droneId, name, hashApiKey(key)],
  );
  console.log(`registered drone '${droneId}'`);
  console.log(`API key (store securely, shown once): ${key}`);
}

main()
  .catch((e) => {
    console.error(e);
    process.exitCode = 1;
  })
  .finally(() => closePool());
