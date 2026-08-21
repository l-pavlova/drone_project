import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// The ops endpoints can require an `x-admin-key` (ADMIN_API_KEY in web/.env).
// The dashboard is a browser app, so the key must NOT reach it — the dev proxy
// holds it and injects it server-side instead. In a real deployment this is the
// job of whatever fronts the dashboard; here it keeps :5174 working without
// putting a shared secret in a bundle.
function adminKey(): string {
  try {
    const env = readFileSync(resolve(__dirname, "../../.env"), "utf8");
    const line = env.split(/\r?\n/).find((l) => l.startsWith("ADMIN_API_KEY="));
    return line ? line.slice("ADMIN_API_KEY=".length).trim() : "";
  } catch {
    return ""; // no .env yet: the server is open too, so nothing to inject
  }
}

const key = adminKey();

// :5174 so it can run alongside the driver-facing app on :5173. Same proxy to
// the API on :4000 — the admin panel is a separate deployment boundary, not a
// separate backend.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5174,
    proxy: {
      "/api": {
        target: "http://localhost:4000",
        changeOrigin: true,
        headers: key ? { "x-admin-key": key } : undefined,
      },
      "/ws": { target: "ws://localhost:4000", ws: true },
    },
  },
});
