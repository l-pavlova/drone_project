import { existsSync } from "node:fs";
import { dirname, join, parse } from "node:path";
import { config } from "dotenv";

/**
 * Load the nearest .env by walking up from cwd to the filesystem root. Lets the
 * db scripts run under `pnpm --filter` (cwd = packages/db) yet still pick up the
 * monorepo-root .env. Existing process env always wins over the file.
 */
export function loadEnv(): void {
  let dir = process.cwd();
  const { root } = parse(dir);
  while (true) {
    const candidate = join(dir, ".env");
    if (existsSync(candidate)) {
      config({ path: candidate });
      return;
    }
    if (dir === root) return;
    dir = dirname(dir);
  }
}
