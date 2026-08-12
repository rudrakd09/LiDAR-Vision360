# Cloud Backend & Dashboard

## Status

**Implemented, local-only.** `cloud/backend` (FastAPI + SQLAlchemy) and `cloud/dashboard`
(React + TypeScript + Vite). No production cloud deployment, authentication, or managed database
-- see "Do not implement yet" below. Builds on Phase 12 (`perception/src/streaming/`) without any
change to that layer's own behavior for Unity -- see "Architecture."

## Objective

Show the same synthetic scenario, driven by the real perception pipeline, in both the Unity
digital twin and a live web dashboard, staying in sync because both consume the identical
`frame_id`-numbered stream -- not because either one drives the other.

## Architecture

```
scripts/serve_unity_bridge.py
    Simulator scenario -> real pipeline (preprocess/cluster/classify/track/map/collision/clearance)
        -> PerceptionStreamServer.publish()   (unchanged -- see docs/communication.md)
            -> port 5006, one broadcast, N independent TCP clients
                -> Unity (PerceptionTCPClient.cs)          -- existing, unmodified connection behavior
                -> cloud/backend (backend.ingestion.PerceptionIngestor)  -- an ordinary second client
                    -> in-memory latest-state + SQLite/PostgreSQL event log
                    -> FastAPI REST + WebSocket (/ws/live)
                        -> cloud/dashboard
```

**Single source of truth**: the backend is architecturally just another Unity, written in Python
instead of C#, consuming the identical byte stream on the identical port -- not a relay behind
Unity (`Python -> Unity -> Dashboard` was explicitly rejected). Nothing in `cloud/backend` runs a
simulator, a pipeline stage, or generates synthetic data of its own; every value it serves traces
back to the one `serve_unity_bridge.py` process currently streaming. If that process stops, both
Unity and the dashboard independently show a disconnected/stale state -- neither depends on the
other's connection.

## Ingestion

`backend.ingestion.PerceptionIngestor` runs a background thread (not asyncio -- a plain blocking
`socket.recv()` loop, the same reasoning `PerceptionTCPClient.cs`'s own background thread uses: a
slow/stalled/absent server connection must never block the FastAPI event loop). Reuses
`perception.streaming.framing.MessageFramer` and `perception.streaming.protocol.
classify_frame_id`/`is_frame_id_acceptable` directly -- the exact same functions the Python
producer side's own tests exercise -- rather than a third reimplementation of the wire format
(Unity's C# port is the only consumer that *has* to reimplement it, being a different
language/runtime).

Frame ordering, reconnection, and staleness detection all follow the identical policy
`PerceptionTCPClient.cs` already implements: duplicate/out-of-order frames are dropped before
being applied; a dropped connection retries every `streaming_reconnect_interval_s`; connection
state (`connecting`/`connected`/`reconnecting`/`disconnected`) is exposed via `GET /api/status`.

## In-memory state vs. database

- `backend.state.LatestState`: thread-safe, holds the single latest frame plus a bounded ring
  buffer (`Settings.backend_ring_buffer_size`, default 200 frames) and a deduped-by-`track_id`
  roster with a short grace period (`Settings.backend_track_grace_period_s`, mirrors
  `TrackedObjectVisualizer.cs`'s own `removeAfterSeconds`). Backs `/api/latest`, `/api/objects`,
  `/api/tracks`, and a newly-connected WebSocket client's initial snapshot.
- Database (`backend.models_db`): **not** one row per frame (unbounded growth for no benefit) --
  only `SessionRecord` (one per ingested connection) and `CollisionEvent`/`ClearanceEvent` rows,
  written only when the collision `overall_risk` or clearance `overall_status` actually *changes*
  from the previous frame. This is what `/api/events`, `/api/collision-events`,
  `/api/clearance-events`, and `/api/sessions` serve, each capped at `Settings.
  backend_event_max_limit` regardless of the requested `limit` -- "do not expose unlimited
  historical records."

## Database

Tries `Settings.database_url` (PostgreSQL by default) first; on any connection failure at startup
(no local server, wrong credentials, driver not installed), falls back to a local SQLite file at
`Settings.backend_sqlite_fallback_path` (`cloud/backend/data/lidar_vision360.db`) and logs a
warning rather than failing to start -- see `backend.db.resolve_database_url`. No local PostgreSQL
was installed in the environment this was built in, so the SQLite fallback is what actually runs
by default; both paths use the same synchronous SQLAlchemy engine and the same ORM models.

## API

| Endpoint | Returns |
|---|---|
| `GET /api/health` | `{status, uptime_s}` |
| `GET /api/status` | connection state, frame counters, dashboard client count |
| `GET /api/latest` | the latest `PERCEPTION_FRAME` payload (204 if none yet) |
| `GET /api/objects` | this frame's `objects` array |
| `GET /api/tracks` | deduped-by-`track_id` roster (recent, not just this frame) |
| `GET /api/events?limit=N` | combined collision + clearance events, most recent first |
| `GET /api/collision-events?limit=N` | collision `overall_risk` transitions |
| `GET /api/clearance-events?limit=N` | clearance `overall_status` transitions |
| `GET /api/sessions?limit=N` | ingested-connection history |
| `WS /ws/live` | `{type: "snapshot"\|"frame"\|"heartbeat"\|"status"\|"error", ...}` push stream |

## Dashboard

`cloud/dashboard` (React + TypeScript + Vite) -- connects to `/ws/live` for real-time updates and
`/api/*` for its initial/on-demand loads. No hard-coded values; every panel reflects the live
backend state, including an explicit disconnected/empty state when nothing has arrived yet.

## Do not implement yet

Production cloud deployment, authentication, managed/hosted database, mobile app, MQTT,
camera/radar fusion, ML, SLAM, vehicle control -- out of scope for this project's current stage,
same list docs/communication.md's own "Do not implement yet" already established.

## Known limitations

- Synchronous SQLAlchemy, not async -- a deliberate simplification for a local, single-instance
  demo backend's modest write volume; FastAPI runs sync route handlers in a threadpool
  automatically, and the one genuinely real-time path (`/ws/live`) never touches the database.
- `SYSTEM_STATUS` (carrying `source_id`/`scan_rate_hz`) is published once per bridge *run*, not
  once per newly-connecting *client* (a pre-existing Phase 12 behavior, not introduced here) -- a
  backend or Unity client that connects after that message was already sent won't see it until the
  bridge restarts. Both consumers share this limitation identically.
- No authentication on the REST/WebSocket API -- local-prototype scope, explicitly documented, not
  a gap to silently work around.
