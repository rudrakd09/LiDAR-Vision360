# Architecture

## Status

Phase 0 (Foundation), Phase 2 (LiDAR Simulator), Phase 3 (Preprocessing), Phase 4 (Coordinate
Transformation), Phase 5 (Obstacle Clustering), Phase 6 (Geometric Object Classification),
Phase 7 (Object Tracking), Phase 8 (2D Occupancy Grid Mapping), Phase 9 (Collision/Risk Engine),
Phase 10 (Clearance Engine), Phase 11 (Unity Digital Twin), and Phase 12 (Real-Time
Python↔Unity Communication) complete. A local-only cloud backend + dashboard (`cloud/backend`,
`cloud/dashboard`) have also since been added, as a second independent consumer of the same
streaming protocol -- see [docs/cloud.md](cloud.md). This document covers the current,
actually-implemented state plus the target end-state architecture; it will be extended as later
phases land.

## Target end-state pipeline

```
2D 360° LiDAR
    ↓
  STM32
    ↓
  UART
    ↓
PC / Edge Computer
    ↓
LiDAR Perception Engine  (perception/)
    ↓
    ├──────────────┬──────────────┐
    │              │              │
 Unity          Cloud          Alerts
 Digital Twin    Dashboard
 (unity/)       (cloud/)
```

**Unity digital twin (Phase 11) + real-time streaming (Phase 12, both implemented)**:
`scripts/serve_unity_bridge.py` runs the full Python pipeline against a simulator scenario and
serves it to Unity over two independent, non-blocking TCP servers
(`perception/src/streaming/server.py`) -- the original raw `<START>`/`<END>` protocol (unchanged,
port `5005`) and a versioned, enveloped, structured JSON protocol (port `5006`,
`message_type`-dispatched: `PERCEPTION_FRAME`/`HEARTBEAT`/`SYSTEM_STATUS`/`ERROR`) carrying
tracked objects, collision risk, occupancy map, and vehicle/config data. The perception pipeline
never blocks on a slow/stalled Unity client (bounded per-client queues, drop-oldest) -- see
[docs/communication.md](communication.md) for the full protocol/reliability/performance
reference. Unity performs **no** perception computation itself -- see [docs/unity.md](unity.md)
"Important design decision" for the Unity-side architecture, coordinate-system mapping, and setup
instructions.

See [PROJECT_SPECIFICATION.md](../PROJECT_SPECIFICATION.md) for the full phase-by-phase plan.

**Sensor limitation:** the target sensor is a 2D 360° LiDAR. The system does not perform true 3D
reconstruction; Unity renders a 3D digital-twin *representation* of a 2D-LiDAR-derived
environment. True 3D perception is a future extension.

## Current (Phase 0 + Phase 2 + Phase 3 + Phase 4 + Phase 5 + Phase 6 + Phase 7 + Phase 8 + Phase 9 + Phase 10) data flow

```
simulator.SimulatedLiDARDataSource.read_scan()      (ray-casts an Environment of obstacles)
    → ScanFrame { points: [LiDARPoint, ...] }
    ↕ (same shape, replayable)
simulator.RecordedLiDARDataSource.read_scan()        (replays a recorded .jsonl file)
        |
preprocessing.Preprocessor.process()                (validation, outliers, noise/temporal filter)
        |
PreprocessedScan { points: [LiDARPoint, ...], quality_statistics, ... }
        |
coordinates.CoordinateTransformer.transform()       (vectorized polar -> Cartesian)
        |
CartesianScan { points: [CartesianPoint, ...], quality_statistics }
        |
        ├──────────────────────────────────────────────────────┐
        |                                                       |
clustering.DBSCANClusterer.cluster()                  mapping.OccupancyGridMapper.update()
        |                                             (ray traversal + log-odds update, Phase 8)
ClusteredScan { clusters: [ObstacleCluster, ...] }             |
        |                                             OccupancyGrid { cell_states, log_odds, ... }
objects.GeometricClassifier.classify()                (persists across scans -- FREE/OCCUPIED/UNKNOWN)
        |
ClassifiedScan { objects: [DetectedObject, ...], noise_points, ... }
        |
tracking.ObjectTracker.update()                     (nearest-neighbour association + Kalman filter)
        |
TrackedScan { objects: [DetectedObject, ...] (track_id/velocity/direction populated), noise_points, ... }
        |
collision.CollisionRiskEngine.evaluate()            (TTC, collision prediction, SAFE/WARNING/CRITICAL)
        |
CollisionAssessment { results: [CollisionRiskResult, ...], overall_risk, most_critical_object, ... }

clearance.ClearanceEngine.evaluate()                (front/rear/left/right, corridor width, Phase 10 --
        |                                             consumes CartesianScan directly, like mapping,
ClearanceAssessment { front, rear, left, right,      not TrackedScan; see docs/collision.md
                       min_clearance_m, ... }         "Directional clearance")
```

**Mapping (Phase 8) branches off `CartesianScan` directly**, in parallel with clustering, rather
than consuming `TrackedScan` -- an occupancy grid's FREE cells come from every ray's empty
middle, most of which never becomes part of any cluster/object/track at all; see
docs/mapping.md "Architecture" for the full reasoning. **Collision (Phase 9) resumes the linear
chain after tracking**, consuming `TrackedScan` directly -- but deliberately has zero dependency
on the occupancy grid (Phase 8) itself, per this phase's own instruction not to make collision
assessment dependent exclusively on the grid; see docs/collision.md "Architecture". **Clearance
(Phase 10) also branches off `CartesianScan` directly**, alongside mapping and clustering, for the
same reason mapping does -- directional clearance is a property of every LiDAR return, not just
tracked objects; see docs/collision.md "Directional clearance". **Unity (Phase 11) and the local
cloud backend (`cloud/backend`) both consume every stage's output (collision assessment included)
over the Phase 12 real-time streaming protocol** (`perception/src/streaming/`, served by
`scripts/serve_unity_bridge.py`) -- as two independent TCP clients of the same broadcast, not one
relaying to the other; see docs/communication.md, docs/unity.md, and docs/cloud.md.
`scripts/run_foundation_demo.py` exercises the Phase 0 minimal
placeholder end to end; `simulator.cli` (`python -m simulator.cli run ...`) exercises the full
Phase 2 simulator; `scripts/compare_raw_processed.py` and `scripts/benchmark_preprocessing.py`
exercise Phase 3; `scripts/visualize_cartesian.py` and `scripts/benchmark_coordinates.py`
exercise Phase 4; `scripts/visualize_clusters.py` and `scripts/benchmark_clustering.py` exercise
Phase 5; `scripts/visualize_classification.py`, `scripts/benchmark_classification.py`, and
`scripts/evaluate_classification.py` exercise Phase 6; `scripts/visualize_tracking.py`,
`scripts/benchmark_tracking.py`, and `scripts/evaluate_tracking.py` exercise Phase 7;
`scripts/visualize_mapping.py`, `scripts/benchmark_mapping.py`, and `scripts/evaluate_mapping.py`
exercise Phase 8; `scripts/visualize_collision.py`, `scripts/benchmark_collision.py`, and
`scripts/evaluate_collision.py` exercise Phase 9; `scripts/serve_unity_bridge.py` (Phase 11
Unity + Phase 12 streaming) and `scripts/benchmark_streaming.py` (Phase 12) exercise the
Python↔Unity communication layer -- all against simulator scenarios. See
[docs/simulation.md](simulation.md), [docs/preprocessing.md](preprocessing.md),
[docs/coordinates.md](coordinates.md), [docs/clustering.md](clustering.md),
[docs/object-classification.md](object-classification.md), [docs/tracking.md](tracking.md),
[docs/mapping.md](mapping.md), [docs/collision.md](collision.md), [docs/unity.md](unity.md), and
[docs/communication.md](communication.md) for the full write-ups.

**Sensor limitation, restated (see "Target end-state pipeline" above):** `mapping`'s occupancy
grid is a 2D LiDAR occupancy map, not a true 3D map -- consistent with the rest of this project's
2D-LiDAR scope. **`collision` is a prototype collision-awareness system, not a certified
automotive safety system** -- see docs/collision.md "Status".

## Sensor source abstraction

**Status: implemented (sensor-input-abstraction groundwork), no perception algorithm changed.**
Formalizes, under one explicit vocabulary, the seam Phase 0 already established
(`datasources.LiDARDataSource` -> `models.scan.ScanFrame`) so a caller never needs to know or care
whether frames came from the simulator or real STM32 hardware:

```
simulator.SimulatorSource ──┐
                             ├──>  SensorFrame  ──>  preprocessing.Preprocessor.process() ──> ...
datasources.STM32Source ────┘      (== ScanFrame)     (existing pipeline, unchanged)
```

- **`SensorFrame`** (`perception/src/datasources/base.py`) -- a plain alias of `models.scan.
  ScanFrame`, not a new/parallel model. There is exactly one canonical "one full 360° sweep"
  schema; every existing pipeline stage already accepts it unchanged.
- **`SensorSource`** (same file) -- a plain alias of `datasources.LiDARDataSource`. Same
  `connect()`/`disconnect()`/`read_scan() -> SensorFrame`/`is_connected()` contract as before;
  `issubclass(X, SensorSource)` and `issubclass(X, LiDARDataSource)` always agree.
- **`simulator.SimulatorSource`** (`simulator/src/simulator/sensor_source.py`) -- wraps
  `scenarios.make_data_source(scenario)` + the existing `SimulatedLiDARDataSource` behind the
  `SensorSource` interface. Delegates every method; adds no new frame-construction logic. All 10
  predefined scenarios (`simulator/scenarios/*.json`) work through it exactly as they did through
  `make_data_source` directly -- see `simulator/tests/test_sensor_source.py`.
- **`datasources.STM32Source`** (`perception/src/datasources/stm32_source.py`, composing
  `perception/src/datasources/stm32/`) -- **Phase 8: architecture implemented, protocol not yet
  known.** Real connection management (`connection_manager.STM32ConnectionManager`, with
  reconnect-with-backoff), byte-stream framing (`framing.FrameCodec`, delimited or fixed-length),
  CRC/checksum validation (`crc`, several standard algorithms), sequence-number/dropped-frame
  detection (`sequence.SequenceValidator`), per-channel health tracking (`health.HealthMonitor`),
  and a `LiDARMessageParser`/`RadarMessageParser` interface pair (`parsers.py`) -- but `connect()`
  raises `STM32ConfigurationError` (never opens a port, never falls back to simulated data) until
  every wire-protocol field (framing, byte order, message-type IDs, CRC algorithm, scaling,
  timestamp format) is actually configured, because PROJECT_SPECIFICATION.md explicitly forbids
  inventing them ahead of the real hardware spec. See docs/hardware-integration.md "Hardware
  integration checklist" for exactly what's still needed. A parsed R121 radar reading is exposed
  via `STM32Source.latest_radar_reading` (`models.radar.RadarReading`) and consumed by
  `fusion.FusionEngine` (Phase 9, see docs/fusion.md) -- authorized as an explicit, deliberate
  reversal of this prototype's earlier "sensor fusion out of scope" stance, tested only against
  synthetic `RadarReading`s (real R121 fusion is not validated; the CAN protocol is still
  unknown).
- **`scripts/sensor_source.py::get_sensor_source()`** -- the composition-root factory
  `scripts/serve_unity_bridge.py` calls instead of constructing either source directly. Reads
  `Settings.data_source` (`DATA_SOURCE` / env var `LIDAR_DATA_SOURCE`, default `"simulation"`; see
  `.env.example`) and returns `SimulatorSource` or `STM32Source` accordingly. Lives in `scripts/`,
  not `perception/`, because it is the one place allowed to depend on both `perception` and
  `simulator` -- `perception` must never import `simulator` (see "Why `perception` and `simulator`
  are separate top-level projects" below), and `STM32Source` lives in `perception` since hardware
  has nothing to do with the simulator package.

Nothing downstream of `SensorFrame` (preprocessing onward, tracking, collision, clearance,
streaming, the cloud backend, the dashboard, Unity) changed to support this -- they already only
ever depended on `ScanFrame`'s shape, never on which concrete data source produced it.

## Session and sequence management

**Status: implemented.** Formalizes "who is the current run" as an explicit, wire-carried
identity, so the Edge computer is unambiguously the single source of truth for it -- no downstream
consumer (backend, dashboard, Unity) invents or infers a session boundary of its own.

```
pipeline.LiveStateBuilder.session_id      (minted once per scripts/serve_unity_bridge.py run)
    -> streaming.protocol._envelope       (session_id + source_id at the envelope level, EVERY
                                            message type -- PERCEPTION_FRAME/HEARTBEAT/
                                            SYSTEM_STATUS/ERROR alike, not just frames)
    -> backend.ingestion.PerceptionIngestor._apply_session_boundary_if_needed
           - same session_id            -> dispatch normally
           - new session_id             -> LatestState.reset_for_new_session() FIRST, then dispatch
           - a PREVIOUSLY-superseded id -> rejected outright, never applied
    -> backend.state.LatestState (latest_frame/tracks/tracking-history/session_frames_received
       all cleared by reset_for_new_session -- not left to be gradually overwritten by new frames)
    -> GET /debug/live-frame, GET /api/latest, /ws/live snapshot -- never serve a stale
       previous-session frame, even during the brief window before the new session's first
       real frame arrives
    -> dashboard useLiveSocket (lib/sessionOrdering.classifyIncomingFrame) + Unity
       PerceptionTCPClient (SessionValidator) -- each independently rejects a message from an
       already-superseded session on its own merits, the same defense-in-depth pattern this
       project already applies to frame_id (streaming.protocol.classify_frame_id /
       FrameIdValidator.cs) -- three languages, one rule, kept in sync deliberately.
```

**Every run has all four identity fields** (`session_id`, `source_id`, `timestamp`,
`sequence_number`) on every message: `source_id` is `f"simulated:{scenario_id}"` for a simulation
run (`simulator.SimulatorSource`) or `Settings.hardware_source_id` (`"stm32_hardware"` by default)
for a hardware run (`datasources.STM32Source`) -- never the CLI's raw `--scenario` string, which
isn't meaningful in hardware mode (a bug this fixed: `scripts/serve_unity_bridge.py` previously
labelled `SYSTEM_STATUS.source_id` from `args.scenario` directly).

**Frame counters, split by scope, deliberately**: `ConnectionInfo.frames_received`/
`duplicate_or_out_of_order_dropped` stay cumulative for the whole backend *process* lifetime
(existing, tested behavior -- some diagnostics genuinely want "how many messages has this backend
process ever ingested, across every reconnect"); `session_frames_received` resets to 0 on every
new session boundary and is what answers "how many frames has the CURRENT session sent." Both are
real counters, never conflated.

**`GET /debug/stream-status`** (`backend/routes/debug.py`) is the single-call summary of all of
the above: `session_id`, `source_id`, `last_sequence`, `last_timestamp`, `frame_age_ms`, a
`scan_rate_hz` that prefers the Edge's own real per-scan measurement
(`performance_metrics.measured_scan_rate_hz`, from `LiveStateBuilder`) over the possibly-stale
SYSTEM_STATUS configured-target value, `objects`/`tracks`, `backend_status`, `edge_status`
(backend<->bridge TCP state), `websocket_status`/`dashboard_clients_connected`, dropped-frame
count, and `latency_ms` (same-machine-clock `last_message_at - last_transmission_timestamp`, see
docs/communication.md "Latency measurement").

**Sensor status and performance metrics are now on the wire too** -- `PERCEPTION_FRAME.data.
sensor_status` (`{"lidar": {...real...}, "radar": null}` -- `null` because no radar sensor exists
in this project, never a fabricated reading) and `data.performance_metrics`
(`pipeline_processing_ms`/`measured_scan_interval_s`/`measured_scan_rate_hz`/`scans_processed`,
all real, directly measured by `scripts/serve_unity_bridge.py` around its own pipeline stage
calls) -- both sourced from `LiveState`, both additive (older consumers that don't know these keys
exist simply ignore them). The dashboard (`useMeasuredScanRate`) and Unity (`HUDController`) now
prefer this real, Edge-measured rate over their own client-side arrival-timing estimate, which
remains only as a fallback/cross-check for a payload that predates the field.

## Dashboard and Unity as pure LiveState consumers

**Status: implemented.** Dashboard and Unity render `LiveState` (via the wire); neither
detects, classifies, tracks, or computes TTC/clearance/risk. This section covers what changed to
make that literally true end-to-end, not just true of the perception pipeline itself.

```
Edge (pipeline.LiveStateBuilder)
    |
    v
LiveState.tracked_objects   -- track_id, classification, confidence, x/y, velocity,
                                sensor_source, first_seen/last_seen/frames_tracked, trajectory,
                                ttc, risk -- ALL joined/computed at the Edge, by track_id
LiveState.events            -- track_created/track_lost/tracking_state_changed/ttc_change/
                                collision/clearance transitions, detected at the Edge
    |
    v  (serialization.unity_protocol.build_frame_message -- additive `data.tracked_objects`/
    |   `data.events` fields; `data.events` carries only THIS scan's own new events, not the
    |   full retained backlog -- see build_events_payload's own docstring)
    v
   ┌──────────────────────┬──────────────────────┐
Dashboard (/ws/live)                          Unity (port 5006, PerceptionTCPClient)
   │                                              │
TrackedObjectsTable/                          TrackedObjectView.ApplyData(obj, origin,
TrackingHistoryPanel render                   trackedState) renders the SAME
tracked_objects[] directly --                 tracked_objects[] entry (joined by
no client-side join against                   track_id) in its in-world label --
risk.results[], no REST                       same TTC/risk text, same track_id,
round-trip for history                        same source
   │                                              │
useLiveEvents.ts buffers                      (Unity does not yet render a
events[] deltas into the                      timeline UI for events -- the
Event Timeline                                data is parsed and available;
                                               display is a documented gap,
                                               see "Known limitations" below)
```

**Why this matters**: before this, the dashboard's `TrackedObjectsTable` joined `objects[]`
against `risk.results[]` by `track_id` *itself* (a harmless-looking client-side join, but still a
second, independent implementation of a join the Edge already performs once), and
`useTrackBookkeeping`/`useTrackTrajectories` reconstructed first-seen/last-seen/trajectory
*client-side* from repeated frame observations -- both retired now that `tracked_objects[]`
already carries the fully-joined, Edge-computed result directly. If Track 12 shows "Vehicle / TTC
1.8s / WARNING" on the dashboard, Unity's label for the same `track_id` reads the identical
`ttc`/`risk` field off the identical wire message -- not a coincidence of two independent
computations agreeing, but the same computation rendered twice.

**Fabricated-risk fixes, on the Unity side** (found during this pass, previously flagged but not
yet fixed):
- `CollisionRiskIndicator.cs`/`HUDController.cs` defaulted to `"safe"` (green) whenever no real
  `risk` data existed yet (before the first frame, or a bridge run without the collision stage
  wired in) -- a vehicle-body safety indicator and HUD both claiming "confirmed safe" with nothing
  behind that claim. Both now default to `"unknown"` (a distinct gray state), matching the
  `ClearancePanel -> "No clearance data yet"` precedent the dashboard already established.
- `LidarCubes.cs` (legacy raw point-cloud path, no risk data on that wire at all) colored points
  red/orange/yellow/green by distance -- a self-invented pseudo-risk scheme on a channel that
  never carries Python's real risk assessment. Replaced with a neutral near/far shading gradient;
  its stale `> 70f` max-range check (unrelated to this project's actual ~12m sensor) is now an
  externalized, configurable field defaulting to the real value.
- `RiskAudioController.cs`'s documented distance-only fallback (used only when `frame.risk` is
  null) now reads its thresholds from `frame.config` -- the exact same
  `collision_warning_distance_m`/`collision_critical_distance_m` Python's real engine uses --
  instead of a second, independently-configured Inspector duplicate that could silently drift out
  of sync with it.
- `LidarBeep.cs` (legacy audio, pre-existing/deliberately unmodified since Phase 11) still has its
  own disconnected thresholds -- left as-is this pass, consistent with that established
  precedent; flagged, not fixed.

**Known limitations**: Unity changes are written, not compiled/run (no Editor in this
environment -- see docs/unity.md "Status", a pre-existing project-wide constraint). Unity parses
`tracked_objects`/`events` and displays per-track TTC/risk in-world; it does not yet have its own
Event Timeline UI panel (the dashboard's is the reference implementation) -- `events` is received
and available for a future HUD addition. `sensor_status` is parsed by the dashboard but not added
to Unity's C# types (not currently rendered anywhere in Unity) -- deliberate scope trim.

## PostgreSQL as history, WebSocket as live transport

**Status: implemented.** PostgreSQL (or its local SQLite fallback -- same schema, see
`backend.db.resolve_database_url`) is this project's PERSISTENCE/HISTORY layer only. It is never
the live transport: `/ws/live` (backed by `state.LatestState`'s in-memory ring buffer) is, and
stays, the only path a dashboard's real-time render depends on -- no route ever blocks a live
value on a database read.

**What's persisted** (`backend/models_db.py`), each row carrying `session_id`/`source_id`/
`timestamp`:

| Table | What | Written |
|---|---|---|
| `sessions` | One row per ingested TCP connection | Open on connect, closed (`ended_at`/`status`) on disconnect |
| `tracks` | One row per unique `track_id` ever seen this session | Created on `track_created`, updated in-memory every frame, flushed on `track_lost`/session close -- **never one row per frame** |
| `collision_events` | `overall_risk` transitions (+ `collision_predicted`) | Sourced from the Edge's own `LiveState.events`, enriched from `risk.most_critical` |
| `clearance_events` | `overall_status` transitions | Sourced from `LiveState.events`, enriched from `clearance` |
| `ttc_events` | Per-track approaching/not-approaching transitions | Sourced from `LiveState.events`, enriched from `risk.results[]` |
| `sensor_events` | LiDAR data-quality transitions (`Settings.sensor_quality_degraded_threshold_percent`) | Sourced from `LiveState.events` |

**Not persisted**: raw `PERCEPTION_FRAME`s/point clouds (unbounded growth for no benefit -- "recent
state" is `state.LatestState`'s bounded ring buffer); `tracking_state_changed` events (routine
TENTATIVE->CONFIRMED graduations, not a safety-relevant moment -- still broadcast live and visible
in `LiveState.events` on the wire, just not a separate DB row).

**Events are sourced from the Edge, never re-derived** -- `ingestion.PerceptionIngestor.
_persist_events_from_frame` reads `data["events"]` (`pipeline.LiveStateBuilder`'s own output,
already on the wire -- see "Dashboard and Unity as pure LiveState consumers" above) and persists
exactly what the Edge already decided, enriched with same-frame detail for the richer DB columns.
The backend previously re-derived transitions itself from raw `data.risk`/`data.clearance`
(`_last_risk_level`/`_last_clearance_status`); that duplicate logic is now removed.

**New REST endpoints**: `GET /ttc-events`, `GET /sensor-events`, `GET /tracks-history` (mirroring
`GET /collision-events`/`GET /clearance-events`'s own session-scoped-by-default, capped-limit
pattern); `GET /events` now also folds in TTC/sensor transitions.

## Real-time performance monitoring

**Status: implemented, all figures measured from real timestamps.** `GET /debug/stream-status`
now also reports `sensor_ingestion_latency_ms` (`last_message_at - frame.timestamp` -- sensor
capture through the Edge's own pipeline processing to backend receipt). The dashboard's `/ws/live`
"frame" messages now carry `broadcast_at` (stamped by `ingestion.py` at broadcast time); `useLiveSocket`
computes `websocketLatencyMs` (`Date.now() - broadcast_at`) and `endToEndLatencyMs`
(`Date.now() - frame.timestamp`) itself, from these two real timestamps -- never assumed,
never a fixed/fabricated figure. Combined with the already-real `measured_scan_rate_hz`
(`LiveState.performance_metrics`) and `duplicate_or_out_of_order_dropped` (backend-counted), the
Live Data panel's full field list -- sensor ingestion latency, processing latency
(`pipeline_processing_ms`), WebSocket latency, end-to-end latency, frame age, scan rate, dropped
frames -- is entirely measured, matching this project's own "no fake 10Hz/0ms/0 dropped frames
unless actually measured" rule.

## Repository layout

The implemented layout follows `PROJECT_SPECIFICATION.md` with two justified additions inside
`perception/src/`:

- **`common/`** — configuration (`Settings`, env-var driven) and logging setup. Needed by every
  pipeline stage from Phase 0 onward; the original proposed tree had no place for cross-cutting
  infrastructure, so it was added as its own package rather than duplicated into `models/` or
  `pipeline/`.
- **`datasources/`** — the `LiDARDataSource` abstraction (`SimulatedLiDARDataSource` today,
  `SerialLiDARDataSource` placeholder for Phase 16). The spec's Phase 16 asks for exactly this
  abstraction; it's introduced now (Phase 0) because the rest of the pipeline needs *something*
  to pull `ScanFrame`s from, and building the seam early keeps everything else
  hardware-independent from day one.

`preprocessing`, `coordinates`, `clustering`, `objects`, `tracking`, `mapping`, `collision`, and
`clearance` are now implemented (Phases 3-10). `perception/src/pipeline/` still exists as an
empty, documented placeholder matching the proposed tree exactly -- each pipeline stage today is
instead wired up directly by its own caller (`scripts/serve_unity_bridge.py`); a shared
orchestration module remains a reasonable future refactor once a second caller needs the exact
same stage sequence, but isn't required by anything that exists yet.

Two more `perception/src/*` packages exist beyond `PROJECT_SPECIFICATION.md`'s original proposed
tree, added for the same "the pipeline needs *something* concrete here, and no existing package
is the right home" reasoning as `common`/`datasources` above: **`serialization/`** (Phase 11) --
translates this project's own canonical `models.*` types into JSON-safe dicts for an external
consumer (Unity today; a future Phase 13+ cloud backend would reuse the same translation layer
rather than reinventing it) without that consumer ever needing perception logic of its own -- and
**`streaming/`** (Phase 12) -- the non-blocking TCP server/queueing/framing/versioned-envelope
infrastructure that actually delivers those messages in real time. Both are deliberately outside
every pipeline-*stage* package (they are not perception algorithms) but inside `perception/src/`
rather than e.g. `unity/`, since Python remains the sole owner of what gets sent and how it's
versioned -- see docs/communication.md "Architecture principle".

`simulator/src/` deliberately uses a single cohesive package (`simulator/src/simulator/`) rather
than `perception`'s flat multi-package style. `perception/src/*` splits into many top-level
packages because each (clustering, tracking, collision, ...) is an independent pipeline stage
built/tested in its own phase; `simulator`'s pieces (geometry, obstacles, environment, noise,
scenarios, recording, cli, visualize) are all one subsystem delivered in a single phase, so they
stay grouped under one importable name.

```
LiDAR-Vision360/
├── perception/          Python perception engine
│   ├── src/
│   │   ├── models/          canonical data models                    [Phase 0 - done]
│   │   ├── common/          config + logging (justified addition)    [Phase 0 - done]
│   │   ├── datasources/     LiDARDataSource abstraction               [Phase 0 - minimal impl]
│   │   ├── preprocessing/   validation/range/outlier/noise filtering  [Phase 3 - done]
│   │   ├── coordinates/     polar -> Cartesian (vectorized)           [Phase 4 - done]
│   │   ├── clustering/      DBSCAN obstacle clustering                [Phase 5 - done]
│   │   ├── objects/         geometric shape classification            [Phase 6 - done]
│   │   ├── tracking/        cross-frame tracking + Kalman filter      [Phase 7 - done]
│   │   ├── mapping/         occupancy grid                            [Phase 8 - done]
│   │   ├── collision/       collision-risk / TTC engine               [Phase 9 - done]
│   │   ├── clearance/       directional clearance engine              [Phase 10 - done]
│   │   ├── serialization/   canonical models -> JSON-safe payloads     [Phase 11 - done]
│   │   ├── streaming/       non-blocking TCP servers + envelope        [Phase 12 - done]
│   │   └── pipeline/        end-to-end orchestration                  [not implemented -- not required yet, see "Repository layout" above]
│   └── tests/
├── simulator/            full-featured LiDAR scenario simulator      [Phase 2 - done]
│   ├── src/simulator/       geometry/obstacles/environment/noise/datasource/recording/scenarios/cli/visualize
│   ├── scenarios/            10 predefined scenario JSON files
│   └── tests/
├── unity/LiDARVision360/ Unity digital twin                          [Phase 11 - done, extended Phase 12]
├── cloud/backend/        FastAPI + SQLAlchemy telemetry/API           [local-only -- see docs/cloud.md]
├── cloud/dashboard/      React + TypeScript dashboard                [local-only -- see docs/cloud.md]
├── embedded/stm32/       STM32 firmware                              [Phase 16, hardware]
├── docs/                 documentation (this file and siblings)
├── scripts/              setup/demo/dev scripts
└── docker/               containerization                            [as needed]
```

## Why `perception` and `simulator` are separate top-level projects

`perception/` must work against *either* simulated or real hardware data through
`LiDARDataSource`, so it cannot depend on `simulator/`'s scenario-authoring internals (walls,
moving obstacles, noise profiles). `simulator/` (Phase 2) depends on `perception` being installed
first -- it imports `perception`'s `models`, `common`, and `datasources` packages directly, the
same way `perception`'s own internal modules cross-import each other -- and implements
`datasources.LiDARDataSource` from perception, producing output in exactly the
`ScanFrame`/`LiDARPoint` shape `perception/src/models` defines. This keeps the dependency
one-directional (`simulator` → `perception`, never the reverse), so the perception pipeline never
needs to know a simulator exists.

## Cross-package integration tests

A package's own test suite never depends on a package that depends on *it*. `perception`
(including `preprocessing`) has no dependency on `simulator`, so `perception/tests/` never
imports `simulator` either -- even for integration testing. Where a phase's spec explicitly asks
for integration tests against simulator scenarios (Phase 3's preprocessing, and future phases the
same way), those tests live in `simulator/tests/` instead, since `simulator` already depends on
`perception`. This keeps the dependency graph one-directional in both code and tests.

## Configuration

All environment-, deployment-, or hardware-specific values (LiDAR range, point count, vehicle
dimensions, backend host/port, database URL, log level) live on `common.config.Settings`
(pydantic-settings), overridable via environment variables prefixed `LIDAR_` or a root-level
`.env` file (see `.env.example`). No such value is hard-coded in pipeline logic.
