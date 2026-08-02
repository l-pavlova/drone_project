import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// :5174 so it can run alongside the driver-facing app on :5173. Same proxy to
// the API on :4000 — the admin panel is a separate deployment boundary, not a
// separate backend.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5174,
    proxy: {
      "/api": { target: "http://localhost:4000", changeOrigin: true },
      "/ws": { target: "ws://localhost:4000", ws: true },
    },
  },
});
