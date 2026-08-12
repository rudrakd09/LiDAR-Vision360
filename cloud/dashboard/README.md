# Dashboard

React + TypeScript + Vite live dashboard for LiDAR-Vision360. Connects to `cloud/backend`'s REST +
WebSocket API (`/api/*`, `/ws/live`) -- never directly to the Python bridge or to Unity. See
[docs/cloud.md](../../docs/cloud.md).

## Install

```bash
cd cloud/dashboard
npm install
```

## Run (dev)

```bash
# in another terminal: python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle
# in another terminal: uvicorn backend.main:app --app-dir ../backend/src --host 0.0.0.0 --port 8000
npm run dev
```

Open the printed local URL (default `http://localhost:5173`).

## Build

```bash
npm run build
npm run preview
```
