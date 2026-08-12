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
