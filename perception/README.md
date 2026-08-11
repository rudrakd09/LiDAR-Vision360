# perception

The Python perception engine for LiDAR-Vision360: everything between "a stream of `(angle,
distance)` measurements" and "a structured description of the environment" (obstacles, tracking,
occupancy, collision/clearance risk). Runs identically against simulated or real LiDAR data via
the `datasources.LiDARDataSource` abstraction.

## Status

**Phase 0 / Foundation — implemented.**

| Package | Purpose | Status |
|---|---|---|
| `src/models` | Canonical data models (`LiDARPoint`, `CartesianPoint`, `DetectedObject`, `ScanFrame`, ...) | ✅ Implemented |
| `src/common` | Configuration (`Settings`) and logging setup | ✅ Implemented |
| `src/datasources` | `LiDARDataSource` abstraction + `SimulatedLiDARDataSource` (minimal placeholder) + `SerialLiDARDataSource` (stub) | ✅ Implemented (minimal). The full-featured simulator now lives in [`../simulator/`](../simulator/) (Phase 2) and implements this same interface. |
| `src/preprocessing` | Range validation, outlier/noise filtering | ⏳ Phase 3 |
| `src/coordinates` | Polar → Cartesian conversion | ⏳ Phase 4 |
| `src/clustering` | DBSCAN-based obstacle clustering | ⏳ Phase 5 |
| `src/objects` | Geometry-based shape classification | ⏳ Phase 6 |
| `src/tracking` | Cross-frame association + Kalman filtering | ⏳ Phase 7 |
| `src/mapping` | 2D occupancy grid | ⏳ Phase 8 |
| `src/collision` | Collision-risk / Time-to-Collision engine | ⏳ Phase 9 |
| `src/clearance` | Front/rear/left/right clearance engine | ⏳ Phase 10 |
| `src/pipeline` | End-to-end orchestration of all of the above | ⏳ Assembled incrementally |

Note: the full-featured LiDAR simulator (configurable walls, poles, moving obstacles, noise
profiles, ...) lives in `../simulator/` and is Phase 2. `SimulatedLiDARDataSource` here is
intentionally minimal — it exists to make the `LiDARDataSource` abstraction runnable and testable
today.

## Setup

From the repository root:

```bash
python -m venv .venv
```

Activate it, then install the perception package in editable mode with dev dependencies:

```bash
pip install -e "./perception[dev]"
```

(Windows PowerShell activation: `.venv\Scripts\Activate.ps1`. macOS/Linux: `source .venv/bin/activate`.)

This makes every `src/*` subpackage (`models`, `common`, `datasources`, `preprocessing`, ...)
importable at the top level, e.g. `from models import LiDARPoint`.

Copy `.env.example` (repository root) to `.env` and adjust values if you want non-default
configuration; every setting has a working default even without a `.env` file.

## Running tests

```bash
pytest perception/tests
```

or, from inside `perception/`:

```bash
pytest
```

## Running the foundation demo

A minimal end-to-end smoke test of the foundation (config → logging → simulated data source →
one scan frame):

```bash
python scripts/run_foundation_demo.py
```

(run from the repository root, with the virtualenv active).

## Coordinate convention

`angle` is in degrees, `[0, 360)`, measured counter-clockwise from the vehicle's forward axis.
`distance` is in meters. Full polar → Cartesian conversion details will be documented in
`docs/data-model.md` and `docs/coordinates.md`-equivalent content once Phase 4 lands (see
`docs/simulation.md`/`docs/perception.md` placeholders in the meantime).

## Design notes / deviations from the proposed tree

`common/` and `datasources/` are additions to the `src/` layout proposed in
`PROJECT_SPECIFICATION.md` (which lists `models/preprocessing/coordinates/clustering/objects/
tracking/mapping/collision/clearance/pipeline`). Rationale is recorded in
[`../docs/architecture.md`](../docs/architecture.md).
