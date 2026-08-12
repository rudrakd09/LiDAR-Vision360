import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Backend defaults to 127.0.0.1:8000 (Settings.backend_host/backend_port -- see
// perception/src/common/config.py). Dashboard dev server defaults to 5173, matching
// Settings.backend_cors_origins' own default so the backend accepts requests from here with no
// extra configuration.
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
  },
});
