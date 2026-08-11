# perception

The Python perception engine for LiDAR-Vision360: everything between "a stream of `(angle,
distance)` measurements" and "a structured description of the environment" (obstacles, tracking,
occupancy, collision/clearance risk). Runs identically against simulated or real LiDAR data via
the `datasources.LiDARDataSource` abstraction.

## Status

**Phase 0 / Foundation, Phase 3 / Preprocessing, Phase 4 / Coordinate Transformation, and Phase 5 / Obstacle Clustering — implemented.**

| Package | Purpose | Status |
|---|---|---|
| `src/models` | Canonical data models (`LiDARPoint`, `CartesianPoint`, `DetectedObject`, `ScanFrame`, `PreprocessedScan`, `CartesianScan`, `ObstacleCluster`, `ClusteredScan`, ...) | ✅ Implemented |
| `src/common` | Configuration (`Settings`) and logging setup | ✅ Implemented |
| `src/datasources` | `LiDARDataSource` abstraction + `SimulatedLiDARDataSource` (minimal placeholder) + `SerialLiDARDataSource` (stub) | ✅ Implemented (minimal). The full-featured simulator now lives in [`../simulator/`](../simulator/) (Phase 2) and implements this same interface. |
| `src/preprocessing` | Validation, range filtering, outlier detection, noise + optional temporal filtering | ✅ Implemented. See [`../docs/preprocessing.md`](../docs/preprocessing.md). |
| `src/coordinates` | Polar → Cartesian conversion (vectorized) | ✅ Implemented. See [`../docs/coordinates.md`](../docs/coordinates.md). |
| `src/clustering` | DBSCAN-based obstacle clustering | ✅ Implemented. See [`../docs/clustering.md`](../docs/clustering.md). |
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

## Preprocessing

```python
from preprocessing import Preprocessor

preprocessor = Preprocessor()
clean_scan = preprocessor.process(raw_scan_frame)  # ScanFrame -> PreprocessedScan
```

See [`../docs/preprocessing.md`](../docs/preprocessing.md) for the full write-up (algorithms,
configuration, quality metrics, performance, limitations), and
`scripts/compare_raw_processed.py` / `scripts/benchmark_preprocessing.py` at the repository root
for debug/benchmark tooling run against simulator scenarios.

## Coordinate transformation

```python
from coordinates import CoordinateTransformer

transformer = CoordinateTransformer()
cartesian_scan = transformer.transform(clean_scan)  # PreprocessedScan -> CartesianScan
```

`angle` is in degrees, `[0, 360)`, measured counter-clockwise from the vehicle's forward axis
(`0°`=+X/forward, `90°`=+Y/left, `180°`=-X, `270°`=-Y); `distance` is in meters;
`x = distance*cos(radians(angle))`, `y = distance*sin(radians(angle))`. See
[`../docs/coordinates.md`](../docs/coordinates.md) for the full write-up, and
`scripts/visualize_cartesian.py` / `scripts/benchmark_coordinates.py` for debug/benchmark tooling.

## Obstacle clustering

```python
from clustering import DBSCANClusterer

clusterer = DBSCANClusterer()
clustered_scan = clusterer.cluster(cartesian_scan)  # CartesianScan -> ClusteredScan
```

DBSCAN on `(x, y)`, defaults `eps_m=0.6`/`min_samples=3` (empirically validated against every
simulator scenario, not just theory -- see [`../docs/clustering.md`](../docs/clustering.md)
"Parameter selection"). Points near `lidar_range_max_m` (free space/no-return) are excluded from
clustering and counted as noise. See `../docs/clustering.md` for the full write-up, and
`scripts/visualize_clusters.py` / `scripts/benchmark_clustering.py` for debug/benchmark tooling.

## Design notes / deviations from the proposed tree

`common/` and `datasources/` are additions to the `src/` layout proposed in
`PROJECT_SPECIFICATION.md` (which lists `models/preprocessing/coordinates/clustering/objects/
tracking/mapping/collision/clearance/pipeline`). Rationale is recorded in
[`../docs/architecture.md`](../docs/architecture.md).
