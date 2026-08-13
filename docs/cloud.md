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

## Tracking history

Every `objects[]` entry a frame carries already comes from `tracking.ObjectTracker` (Phase 7) --
`LatestState` additionally remembers, per `track_id`, a bounded deque (`Settings.
backend_track_history_length`, default 50 points) of `{frame_id, timestamp, x, y, vx, vy, distance,
classification, tracking_state}`, each copied verbatim from that scan's own object entry. **No
second tracking algorithm** -- this is bookkeeping over what the tracker already produced, not a
re-derivation. Cleared (along with first-seen/last-seen/frame-count bookkeeping) the moment a
track_id is pruned from the roster (`backend_track_grace_period_s` elapsed with no sighting), so a
long-lived demo can't grow this without bound either.

`GET /api/tracks/{track_id}` returns that track's current snapshot plus `first_seen_at`/
`last_seen_at`/`frames_tracked`/`history_length` -- 404 if the id isn't currently known (never
seen, or expired), not a zeroed placeholder. `GET /api/tracking-history?track_id=...&limit=N`
returns the bounded point list itself. `GET /api/tracks` (the roster) already includes the same
first-seen/last-seen/frames-tracked summary fields inline, so a dashboard table doesn't need one
extra request per row just to show them -- only the full point-by-point trajectory (on-demand,
e.g. an expandable table row) needs the separate `/api/tracking-history` call.

## Session lifecycle

`connection.state` (`"disconnected"|"connecting"|"connected"|"reconnecting"`) is the *raw TCP*
state of the ingestion socket -- it stays `"connected"` even if the bridge process on the other end
has stalled without actually closing the connection. `backend.state.compute_session_status`
computes a *separate* concept from that plus `last_message_at`:

```
connection.state != "connected"                                  -> "disconnected"
connection.state == "connected", no message received yet          -> "active"   (fresh connection grace period)
connection.state == "connected", last message <= threshold old    -> "active"
connection.state == "connected", last message  > threshold old    -> "stale"
```

`Settings.backend_session_stale_threshold_s` (default 3.0s) is the threshold. Exposed as
`session_status` on `GET /api/status`, `GET /api/metrics`, and the root `GET /`'s `stream` object --
this is what fixed a real, reported bug where the dashboard showed "Backend->Bridge: connected"
and "Session: no active session" side by side with no way to tell whether that was a contradiction
or two genuinely different facts (it was always two different facts; there was previously no
single field that said so directly).

## API

Every route below is mounted at **both** a bare path (`/health`) and the original `/api/*` path
(`/api/health`) -- the exact same route function, included twice (see `main.py`) -- so existing
`/api/*` callers keep working unchanged while a bare-path caller (or the spec this section was
written against) also works.

| Endpoint | Returns |
|---|---|
| `GET /` | service info: name, version, docs link, full endpoint list, current stream/session status |
| `GET /health` | `{status, uptime_s}` |
| `GET /status` | connection state, `session_status`, frame counters, dashboard client count |
| `GET /latest` | the latest `PERCEPTION_FRAME` payload (204 if none yet) |
| `GET /objects` | this frame's `objects` array |
| `GET /tracks` | deduped-by-`track_id` roster, with first-seen/last-seen/frames-tracked summary fields |
| `GET /tracks/{track_id}` | one track's current snapshot + summary (404 if unknown/expired) |
| `GET /tracking-history?track_id=...&limit=N` | bounded per-scan trajectory points for one track (`track_id` required; 404 if unknown/expired) |
| `GET /events?limit=N` | combined collision + clearance events, most recent first, session-scoped by default |
| `GET /collision-events?limit=N` | collision `overall_risk` transitions |
| `GET /clearance-events?limit=N` | clearance `overall_status` transitions |
| `GET /sessions?limit=N` | ingested-connection history |
| `GET /sessions/{session_id}` | one session's detail (404 if unknown) |
| `GET /metrics` | real, already-tracked counters only (frames received/dropped, active tracks, ring buffer usage, uptime, `session_status`) -- no fabricated throughput/rate figures this backend doesn't actually measure |
| `GET /docs`, `GET /redoc` | FastAPI's own Swagger UI / ReDoc, automatic |
| `WS /ws/live` | `{type: "snapshot"\|"frame"\|"heartbeat"\|"status"\|"error", ...}` push stream |

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
  bridge restarts. `source_id` is no longer actually affected by this in practice (`ingestion.py`
  also reads it from every `PERCEPTION_FRAME`'s own `data.source_id`, always present, so a
  late-connecting backend still learns it immediately) -- `scan_rate_hz` has no such fallback (it
  isn't part of the per-frame payload at all) and is genuinely still `null` for a late-connecting
  client until the bridge restarts.
- Client-side "tracks tracked" bookkeeping shown in the dashboard's expandable table rows
  (`useTrackBookkeeping`) counts frames *this browser tab has observed*, which can differ from the
  backend's own `frames_tracked` (`GET /api/tracks/{id}`) if the tab connected partway through a
  track's life or missed messages during a reconnect -- both numbers are honestly what they say
  they are, just from two different vantage points; neither is "the real" one dressed up as the
  other.
- No authentication on the REST/WebSocket API -- local-prototype scope, explicitly documented, not
  a gap to silently work around.
