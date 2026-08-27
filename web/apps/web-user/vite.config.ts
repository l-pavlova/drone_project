import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// Dev proxy so the SPA talks to the API/WS on :4000 without CORS.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      // **127.0.0.1, NOT localhost.** uvicorn binds IPv4 only; on Windows `localhost`
      // resolves to ::1 first, so Node's proxy pays a ~2 s connect stall before
      // falling back on EVERY request. Measured: /api/v1/summary is 0.035 s direct
      // and 2.032 s through this proxy with a localhost target. That is the map
      // feeling slow, and it is the same root cause as the demo replay taking 301 s
      // for 137 frames (see sim_uplink.py).
      "/api": { target: "http://127.0.0.1:4000", changeOrigin: true },
      "/ws": { target: "ws://127.0.0.1:4000", ws: true },
    },
  },
});
