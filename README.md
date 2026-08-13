# LiDAR-Vision360

**Real-Time LiDAR-Based Vehicle Environment Perception, Digital Twin, Collision-Awareness and
Cloud Monitoring System.**

Full specification: [PROJECT_SPECIFICATION.md](PROJECT_SPECIFICATION.md).

## What this is

A prototype system that turns ~360 `(angle, distance)` measurements from a 2D 360° LiDAR into
structured environmental understanding — obstacle detection, classification, tracking, an
occupancy map, collision-risk and clearance analysis — surfaced through a Unity digital twin and
a cloud dashboard.

**Important sensor limitation:** the target sensor is a **2D 360° LiDAR**. This system does not
perform true 3D LiDAR reconstruction. Unity provides a 3D visualization/digital-twin
*representation* of a 2D-LiDAR-derived environment. True 3D perception is a future extension.

## Target architecture

```
2D 360° LiDAR → STM32 → UART → PC / Edge Computer → LiDAR Perception Engine
                                                            │
                                        ┌───────────────────┼───────────────┐
                                        │                   │               │
                                     Unity              Cloud            Alerts
                                  Digital Twin          Dashboard
                                        │                   │
                                  Environment          Historical
                                  Visualization         Analytics
```

Development proceeds **hardware-independent first**: the perception pipeline, Unity
visualization, cloud backend, and dashboard are all built and tested against a simulated LiDAR
data source before any STM32/hardware integration happens. See
[docs/architecture.md](docs/architecture.md).

## Development principle

Deterministic, geometry-based perception first — no deep learning in this prototype.

```
Simulated LiDAR → Filtering → Coordinate conversion → Clustering → Object properties →
Shape classification → Object tracking → Occupancy mapping → Collision detection →
Clearance analysis → Unity visualization → Cloud backend → Dashboard →
Hardware integration → Advanced ML / sensor fusion
```

## Repository structure

```
LiDAR-Vision360/
├── perception/       Python perception engine (models, filtering, clustering, tracking, ...)
├── simulator/        Configurable 360° LiDAR scenario simulator            [Phase 2 - done]
├── unity/            Unity digital twin                                   [Phase 11]
├── cloud/            FastAPI backend + React/TypeScript dashboard          [Phase 12-14]
├── embedded/stm32/   STM32 firmware                                       [Phase 16, hardware]
├── docs/             Architecture, data model, and per-subsystem docs
├── scripts/          Setup and demo scripts
└── docker/           Containerization                                     [as needed]
```

Each directory has its own `README.md`. See [docs/architecture.md](docs/architecture.md) for the
full rationale, including two justified additions inside `perception/src/` (`common/`,
`datasources/`) beyond the tree originally proposed in the specification.

## Status

**Phases 0–12 complete**, plus a local cloud backend + live dashboard (numbered "Phase 12/13/14"
in the original spec, but see [docs/architecture.md](docs/architecture.md) "Status" for the
as-built phase-number history): Foundation, LiDAR Simulator, Preprocessing, Coordinate
Transformation, Obstacle Clustering, Geometric Object Classification, Object Tracking + Kalman
Filter, 2D Occupancy Grid Mapping, Collision/Risk Engine (TTC + prediction), Directional Clearance
Engine, Unity Digital Twin, real-time Python↔Unity structured streaming, and a local FastAPI
backend + React/TypeScript dashboard consuming that same stream independently of Unity.

Implemented: repository structure, configuration system, logging, canonical data models across
every stage; a full ray-casting 2D 360° LiDAR simulator (`simulator/`) with configurable
environments, noise, moving obstacles, 10 predefined scenarios, real-time pacing, recording/replay,
a 2D debug visualizer, and a CLI; preprocessing (validation, range filtering, outlier/noise
filtering, optional temporal smoothing); vectorized polar→Cartesian coordinate transformation;
DBSCAN obstacle clustering; geometry-based shape classification (`WALL`/`VEHICLE_LIKE`/
`POLE_LIKE`/`LARGE_OBSTACLE`/`PERSON_LIKE`/`UNKNOWN`, explainable, **not** general-purpose object
recognition); nearest-neighbour + Kalman-filter object tracking with persistent track IDs and
velocity/trajectory estimation; a log-odds 2D occupancy grid; a collision/risk engine (TTC,
footprint-intersection prediction, SAFE/WARNING/CRITICAL); a directional clearance engine
(front/rear/left/right, corridor width, SAFE/CAUTION/LOW_CLEARANCE/CRITICAL); the Unity digital
twin (`unity/LiDARVision360`); the real-time structured JSON + legacy raw streaming protocol
(`perception/src/streaming/`, `scripts/serve_unity_bridge.py`); and `cloud/backend` +
`cloud/dashboard` (see "Cloud backend & dashboard demo" below). **972 automated tests total**
(perception 684, simulator 213, cloud backend 59, cloud dashboard 16) — see
[docs/testing.md](docs/testing.md).

Not yet implemented: production cloud deployment/authentication, historical analytics beyond the
bounded event log already in place, configurable alerts, hardware (STM32) integration. See
[PROJECT_SPECIFICATION.md](PROJECT_SPECIFICATION.md) for the full phase list and each package's
README/`docs/` page for per-subsystem detail.

## Quickstart

Requires Python 3.10+.

```bash
python -m venv .venv
```

Activate the environment:

```bash
# Windows PowerShell
.venv\Scripts\Activate.ps1
# macOS/Linux
source .venv/bin/activate
```

Install the perception package, then the simulator (editable, with test dependencies; the
simulator depends on perception being installed first):

```bash
pip install -e "./perception[dev]"
pip install -e "./simulator[dev]"        # add the `viz` extra for the 2D debug plot:
pip install -e "./simulator[dev,viz]"
```

Copy the environment template (optional — every setting has a working default):

```bash
cp .env.example .env
```

Run the test suites (each subproject's tests run independently — see
[docs/testing.md](docs/testing.md)):

```bash
pytest perception/tests
pytest simulator/tests
```

Run the foundation smoke-test demo:

```bash
python scripts/run_foundation_demo.py
```

Run the LiDAR simulator:

```bash
python -m simulator.cli list
python -m simulator.cli run --scenario 02_wall_in_front --scans 5
```

Run preprocessing against a scenario (raw vs. processed comparison, and a performance benchmark):

```bash
python scripts/compare_raw_processed.py --scenario 10_missing_outliers --scans 3
python scripts/benchmark_preprocessing.py
```

Run coordinate transformation against a scenario (2D debug plot, and a performance benchmark):

```bash
python scripts/visualize_cartesian.py --scenario 06_narrow_corridor --visualize
python scripts/benchmark_coordinates.py
```

Run obstacle clustering against a scenario (2D cluster debug plot, and a performance benchmark):

```bash
python scripts/visualize_clusters.py --scenario 05_multiple_obstacles --visualize
python scripts/benchmark_clustering.py
```

Run object classification against a scenario (2D labeled-object plot, explained reasoning, a
performance benchmark, and an accuracy/precision/recall/F1 evaluation against known scenario
ground truth):

```bash
python scripts/visualize_classification.py --scenario 04_vehicle_ahead --visualize --explain
python scripts/benchmark_classification.py
python scripts/evaluate_classification.py
```

`scripts/setup_env.ps1` / `scripts/setup_env.sh` automate all the install steps above (perception
+ simulator, with the `viz` extra).

## Cloud backend & dashboard demo

One command starts the whole stack (Python perception bridge + FastAPI backend + React dashboard),
each in its own window:

```powershell
.\scripts\run_demo.ps1 -Scenario 08_approaching_obstacle -Rate 10
```

Then open `http://localhost:5173`. Unity connects to the same `127.0.0.1:5006` independently (open
`unity/LiDARVision360` in the Editor, `LidarInputManager.mode = StructuredJsonTcp`, press Play) --
Unity and the dashboard consume the identical stream, not one relaying to the other. See
[docs/cloud.md](docs/cloud.md) for the full architecture and API reference.

To run each piece by hand instead (three terminals):

```powershell
python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle --vehicle-speed 0 --rate 10
```
```powershell
uvicorn backend.main:app --app-dir cloud/backend/src --host 0.0.0.0 --port 8000
```
```powershell
cd cloud/dashboard; npm install; npm run dev
```

Other demo-worthy scenarios: `05_multiple_obstacles` (multiple simultaneous tracked objects),
`06_narrow_corridor` (low/critical clearance), `07_moving_crossing` (a laterally-moving, not
head-on, obstacle). `python -m simulator.cli list` shows all 10.

Run every test suite (perception, simulator, backend, dashboard):

```powershell
pytest perception/tests -q
pytest simulator/tests -q
pytest cloud/backend/tests -q
cd cloud/dashboard; npm test; npm run build
```

## Documentation

- [PROJECT_SPECIFICATION.md](PROJECT_SPECIFICATION.md) — full canonical specification
- [docs/architecture.md](docs/architecture.md) — system architecture and repo layout rationale
- [docs/data-model.md](docs/data-model.md) — canonical data models and coordinate convention
- [docs/perception.md](docs/perception.md) — perception pipeline status and stage-by-stage map
- [docs/simulation.md](docs/simulation.md) — LiDAR simulator (Phase 2)
- [docs/preprocessing.md](docs/preprocessing.md) — preprocessing pipeline (Phase 3)
- [docs/coordinates.md](docs/coordinates.md) — coordinate transformation (Phase 4)
- [docs/clustering.md](docs/clustering.md) — obstacle clustering (Phase 5)
- [docs/object-classification.md](docs/object-classification.md) — geometric object classification (Phase 6)
- [docs/tracking.md](docs/tracking.md) — object tracking + Kalman filter (Phase 7)
- [docs/mapping.md](docs/mapping.md) — 2D occupancy grid mapping (Phase 8)
- [docs/collision.md](docs/collision.md) — collision/risk engine + directional clearance (Phases 9–10)
- [docs/unity.md](docs/unity.md) — Unity digital twin (Phase 11)
- [docs/communication.md](docs/communication.md) — real-time Python↔Unity streaming protocol (Phase 12)
- [docs/cloud.md](docs/cloud.md) — local cloud backend + live dashboard
- [docs/hardware-integration.md](docs/hardware-integration.md) — status placeholder until that phase lands
- [docs/testing.md](docs/testing.md) — full test suite breakdown

## License

TBD.
