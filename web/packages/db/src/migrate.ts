import { readdirSync, readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { loadEnv } from "./env.js";
import { closePool, getPool } from "./pool.js";

loadEnv();

const here = dirname(fileURLToPath(import.meta.url));
const migrationsDir = join(here, "..", "migrations");

/**
 * Minimal forward-only migration runner. Applies every *.sql in migrations/
 * (lexicographic order) exactly once, tracked in schema_migrations. Each file
 * runs inside a transaction so a failure rolls back cleanly.
 */
async function main(): Promise<void> {
  const pool = getPool();
  await pool.query(`
    CREATE TABLE IF NOT EXISTS schema_migrations (
      name text PRIMARY KEY,
      applied_at timestamptz NOT NULL DEFAULT now()
    )
  `);

  const files = readdirSync(migrationsDir)
    .filter((f) => f.endsWith(".sql"))
    .sort();

  for (const file of files) {
    const { rowCount } = await pool.query(
      "SELECT 1 FROM schema_migrations WHERE name = $1",
      [file],
    );
    if (rowCount) {
      console.log(`= skip ${file} (already applied)`);
      continue;
    }
    const sql = readFileSync(join(migrationsDir, file), "utf8");
    const client = await pool.connect();
    try {
      await client.query("BEGIN");
      await client.query(sql);
      await client.query("INSERT INTO schema_migrations(name) VALUES ($1)", [file]);
      await client.query("COMMIT");
      console.log(`+ applied ${file}`);
    } catch (err) {
      await client.query("ROLLBACK");
      console.error(`! failed ${file}`);
      throw err;
    } finally {
      client.release();
    }
  }
  console.log("migrations up to date");
}

main()
  .catch((err) => {
    console.error(err);
    process.exitCode = 1;
  })
  .finally(() => closePool());
