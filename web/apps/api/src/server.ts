import { createServer } from "node:http";
import express from "express";
import { config } from "./config.js";
import { devRouter } from "./routes/dev.js";
import { ingestRouter } from "./routes/ingest.js";
import { readRouter } from "./routes/read.js";
import { attachOccupancyWs } from "./ws.js";

const app = express();
app.use(express.json({ limit: "1mb" }));

app.get("/health", (_req, res) => res.json({ ok: true }));
app.use("/api/v1/ingest", ingestRouter);
app.use("/api/v1", readRouter);

// dev/test-only manual occupancy controls
if (process.env.ENABLE_DEV_ROUTES !== "false") {
  app.use("/api/v1/dev", devRouter);
  console.log("dev routes enabled: POST /api/v1/dev/occupy | /free");
}

// eslint-disable-next-line @typescript-eslint/no-unused-vars
app.use((err: unknown, _req: express.Request, res: express.Response, _next: express.NextFunction) => {
  console.error(err);
  res.status(500).json({ error: "internal error" });
});

const server = createServer(app);
attachOccupancyWs(server);

server.listen(config.port, () => {
  console.log(`parkdrone api on :${config.port} (ws /ws/occupancy)`);
});
