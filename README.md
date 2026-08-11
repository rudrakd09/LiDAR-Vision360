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

**Phase 0 — Foundation: complete. Phase 2 — LiDAR Simulator: complete. Phase 3 — Preprocessing: complete.**

Implemented: repository structure, configuration system, logging, canonical data models
(`LiDARPoint`, `CartesianPoint`, `DetectedObject`, `ScanFrame`, `PreprocessedScan`), the
`LiDARDataSource` abstraction, a minimal `SimulatedLiDARDataSource`, a full ray-casting 2D 360°
LiDAR simulator (`simulator/`) with configurable environments, noise, moving obstacles, 10
predefined scenarios, real-time pacing, recording/replay, a 2D debug visualizer, and a CLI; and a
preprocessing pipeline (`perception/src/preprocessing/`) with validation, range filtering, a
circular local-outlier detector, a circular median noise filter, and optional cross-scan temporal
smoothing, plus scan-quality statistics for future health monitoring. **180 automated tests
total.**

Not yet implemented: coordinate conversion, clustering, classification, tracking, occupancy
mapping, collision/clearance engines, Unity, cloud backend/dashboard, alerts, hardware
integration. See [PROJECT_SPECIFICATION.md](PROJECT_SPECIFICATION.md) for the full phase list and
each package's README/`docs/` page for per-subsystem status.

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

`scripts/setup_env.ps1` / `scripts/setup_env.sh` automate all the install steps above (perception
+ simulator, with the `viz` extra).

## Documentation

- [PROJECT_SPECIFICATION.md](PROJECT_SPECIFICATION.md) — full canonical specification
- [docs/architecture.md](docs/architecture.md) — system architecture and repo layout rationale
- [docs/data-model.md](docs/data-model.md) — canonical data models and coordinate convention
- [docs/perception.md](docs/perception.md) — perception pipeline status and stage-by-stage map
- [docs/simulation.md](docs/simulation.md) — LiDAR simulator (Phase 2)
- [docs/preprocessing.md](docs/preprocessing.md) — preprocessing pipeline (Phase 3)
- [docs/clustering.md](docs/clustering.md), [docs/tracking.md](docs/tracking.md),
  [docs/collision.md](docs/collision.md), [docs/unity.md](docs/unity.md),
  [docs/cloud.md](docs/cloud.md),
  [docs/hardware-integration.md](docs/hardware-integration.md) — status placeholders until
  their phase lands
- [docs/testing.md](docs/testing.md) — full test suite breakdown (180 tests)

## License

TBD.
