import { existsSync } from "node:fs";
import { dirname, join, parse } from "node:path";
import { config as dotenvConfig } from "dotenv";

/** Load the nearest .env by walking up from cwd (shared monorepo-root .env). */
function loadEnv(): void {
  let dir = process.cwd();
  const { root } = parse(dir);
  while (true) {
    const candidate = join(dir, ".env");
    if (existsSync(candidate)) {
      dotenvConfig({ path: candidate });
      return;
    }
    if (dir === root) return;
    dir = dirname(dir);
  }
}
loadEnv();

export const config = {
  port: Number(process.env.API_PORT ?? 4000),
  redisUrl: process.env.REDIS_URL ?? "redis://localhost:6379",
  jobsQueue: process.env.PARKDRONE_JOBS_QUEUE ?? "parkdrone:jobs",
  deltasChannel: process.env.PARKDRONE_DELTAS_CHANNEL ?? "parkdrone:deltas",
  s3: {
    endpoint: process.env.S3_ENDPOINT ?? "http://localhost:9000",
    region: process.env.S3_REGION ?? "us-east-1",
    bucket: process.env.S3_BUCKET ?? "parkdrone-frames",
    accessKey: process.env.S3_ACCESS_KEY ?? "minioadmin",
    secretKey: process.env.S3_SECRET_KEY ?? "minioadmin",
  },
  jwtSecret: process.env.JWT_SECRET ?? "dev-change-me",
} as const;
