# Architecture

## Status

Phase 0 (Foundation) complete. This document covers the current, actually-implemented state plus
the target end-state architecture; it will be extended as later phases land.

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

## Current (Phase 0) data flow

```
SimulatedLiDARDataSource.read_scan()
    → ScanFrame { points: [LiDARPoint, ...] }
```

Nothing downstream of the raw scan (filtering, coordinate conversion, clustering, ...) exists yet
-- this is intentionally the entire foundation. `scripts/run_foundation_demo.py` exercises this
path end to end.

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

```
LiDAR-Vision360/
├── perception/          Python perception engine (this phase's focus)
│   ├── src/
│   │   ├── models/          canonical data models                    [Phase 0 - done]
│   │   ├── common/          config + logging (justified addition)    [Phase 0 - done]
│   │   ├── datasources/     LiDARDataSource abstraction               [Phase 0 - minimal]
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
├── simulator/            full-featured LiDAR scenario simulator      [Phase 2]
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
moving obstacles, noise profiles). `simulator/` (Phase 2) will produce output in exactly the
`ScanFrame`/`LiDARPoint` shape `perception/src/models` defines, either by importing
`perception` as a dependency or by being wrapped in a thin `LiDARDataSource` adapter —
decided when Phase 2 starts.

## Configuration

All environment-, deployment-, or hardware-specific values (LiDAR range, point count, vehicle
dimensions, backend host/port, database URL, log level) live on `common.config.Settings`
(pydantic-settings), overridable via environment variables prefixed `LIDAR_` or a root-level
`.env` file (see `.env.example`). No such value is hard-coded in pipeline logic.
