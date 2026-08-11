# Architecture

## Status

Phase 0 (Foundation) and Phase 2 (LiDAR Simulator) complete. This document covers the current,
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

See [PROJECT_SPECIFICATION.md](../PROJECT_SPECIFICATION.md) for the full phase-by-phase plan.

**Sensor limitation:** the target sensor is a 2D 360° LiDAR. The system does not perform true 3D
reconstruction; Unity renders a 3D digital-twin *representation* of a 2D-LiDAR-derived
environment. True 3D perception is a future extension.

## Current (Phase 0 + Phase 2) data flow

```
simulator.SimulatedLiDARDataSource.read_scan()      (ray-casts an Environment of obstacles)
    → ScanFrame { points: [LiDARPoint, ...] }
    ↕ (same shape, replayable)
simulator.RecordedLiDARDataSource.read_scan()        (replays a recorded .jsonl file)
```

Nothing downstream of the raw scan (filtering, coordinate conversion, clustering, ...) exists yet
-- that remains the entire scope through Phase 2. `scripts/run_foundation_demo.py` exercises the
Phase 0 minimal placeholder end to end; `simulator.cli` (`python -m simulator.cli run ...`)
exercises the full Phase 2 simulator. See [docs/simulation.md](simulation.md) for the full
write-up.

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

All other `perception/src/*` packages (`preprocessing`, `coordinates`, `clustering`, `objects`,
`tracking`, `mapping`, `collision`, `clearance`, `pipeline`) exist as empty, documented
placeholders matching the proposed tree exactly; each will be filled in during its corresponding
phase.

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
│   │   ├── preprocessing/   range/outlier/noise filtering             [Phase 3]
│   │   ├── coordinates/     polar -> Cartesian                        [Phase 4]
│   │   ├── clustering/      DBSCAN obstacle clustering                [Phase 5]
│   │   ├── objects/         shape classification                     [Phase 6]
│   │   ├── tracking/        cross-frame tracking + Kalman filter      [Phase 7]
│   │   ├── mapping/         occupancy grid                            [Phase 8]
│   │   ├── collision/       collision-risk / TTC engine               [Phase 9]
│   │   ├── clearance/       clearance engine                          [Phase 10]
│   │   └── pipeline/        end-to-end orchestration                  [ongoing]
│   └── tests/
├── simulator/            full-featured LiDAR scenario simulator      [Phase 2 - done]
│   ├── src/simulator/       geometry/obstacles/environment/noise/datasource/recording/scenarios/cli/visualize
│   ├── scenarios/            10 predefined scenario JSON files
│   └── tests/
├── unity/LiDARVision360/ Unity digital twin                          [Phase 11]
├── cloud/backend/        FastAPI telemetry/API                       [Phase 12]
├── cloud/dashboard/      React + TypeScript dashboard                [Phase 14]
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

## Configuration

All environment-, deployment-, or hardware-specific values (LiDAR range, point count, vehicle
dimensions, backend host/port, database URL, log level) live on `common.config.Settings`
(pydantic-settings), overridable via environment variables prefixed `LIDAR_` or a root-level
`.env` file (see `.env.example`). No such value is hard-coded in pipeline logic.
