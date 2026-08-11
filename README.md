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
├── simulator/        Configurable 360° LiDAR scenario simulator            [Phase 2]
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

**Phase 0 — Foundation: complete.**

Implemented: repository structure, configuration system, logging, canonical data models
(`LiDARPoint`, `CartesianPoint`, `DetectedObject`, `ScanFrame`), the `LiDARDataSource`
abstraction, a minimal `SimulatedLiDARDataSource`, and the base test suite.

Not yet implemented: preprocessing, coordinate conversion, clustering, classification, tracking,
occupancy mapping, collision/clearance engines, Unity, cloud backend/dashboard, alerts, hardware
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

Install the perception package (editable, with test dependencies):

```bash
pip install -e "./perception[dev]"
```

Copy the environment template (optional — every setting has a working default):

```bash
cp .env.example .env
```

Run the test suite:

```bash
pytest perception/tests
```

Run the foundation smoke-test demo:

```bash
python scripts/run_foundation_demo.py
```

`scripts/setup_env.ps1` / `scripts/setup_env.sh` automate the steps above.

## Documentation

- [PROJECT_SPECIFICATION.md](PROJECT_SPECIFICATION.md) — full canonical specification
- [docs/architecture.md](docs/architecture.md) — system architecture and repo layout rationale
- [docs/data-model.md](docs/data-model.md) — canonical data models and coordinate convention
- [docs/perception.md](docs/perception.md), [docs/simulation.md](docs/simulation.md),
  [docs/clustering.md](docs/clustering.md), [docs/tracking.md](docs/tracking.md),
  [docs/collision.md](docs/collision.md), [docs/unity.md](docs/unity.md),
  [docs/cloud.md](docs/cloud.md),
  [docs/hardware-integration.md](docs/hardware-integration.md),
  [docs/testing.md](docs/testing.md) — per-subsystem docs (mostly status placeholders until
  their phase lands)

## License

TBD.
