import { Pool } from "pg";

let pool: Pool | undefined;

/** Shared pg Pool. In production this sits behind PgBouncer (see infra). */
export function getPool(): Pool {
  if (!pool) {
    const connectionString = process.env.DATABASE_URL;
    if (!connectionString) {
      throw new Error("DATABASE_URL is not set");
    }
    pool = new Pool({ connectionString, max: Number(process.env.PG_POOL_MAX ?? 10) });
  }
  return pool;
}

export async function closePool(): Promise<void> {
  if (pool) {
    await pool.end();
    pool = undefined;
  }
}
