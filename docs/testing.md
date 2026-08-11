# Testing

## Status

Phase 0 (Foundation) and Phase 2 (LiDAR Simulator) tests implemented, 101 tests total. Full
coverage across the remaining phases (coordinate conversion, filtering, clustering, object
detection, classification, tracking, Kalman filtering, occupancy grid, collision detection, TTC,
clearance, API, data validation) will be built out as each phase lands, per **Phase 17**.

## Current test suites

Each subproject (`perception/`, `simulator/`) has its own `pyproject.toml`/`testpaths` and is run
independently -- running both `tests/` directories in a single `pytest` invocation from the repo
root currently collides on the shared `tests` package name (both have `tests/__init__.py`), so
run them as two separate invocations, as below.

### `perception/tests/` (24 tests)

- `test_models.py` — validation rules and construction for `LiDARPoint`, `CartesianPoint`,
  `DetectedObject`, `ScanFrame`.
- `test_config.py` — `Settings` defaults and `LIDAR_`-prefixed environment variable overrides.
- `test_datasources.py` — `LiDARDataSource` interface conformance, `SimulatedLiDARDataSource`
  scan generation (point count, angle/distance validity, sequence numbering, context-manager and
  streaming behavior), `SerialLiDARDataSource` stub raising `NotImplementedError` as expected.

### `simulator/tests/` (77 tests)

- `test_geometry.py` — ray/segment and ray/circle intersection math, rectangle corner geometry.
- `test_obstacles.py` — per-obstacle-type (`WallObstacle`, `PoleObstacle`, `RectangleObstacle`)
  distance calculation and motion.
- `test_environment.py` — empty environment, nearest-obstacle-wins with multiple obstacles, range
  limits, obstacle stepping.
- `test_noise.py` — Gaussian noise, outliers, missing measurements, and determinism/divergence
  under fixed/differing random seeds.
- `test_datasource.py` — end-to-end scan generation, exact wall distance with zero noise,
  moving-obstacle distance deltas between consecutive scans, real-time pacing.
- `test_scenarios.py` — all 10 predefined scenarios load and produce valid scans; deterministic
  reproduction of a seeded scenario; the missing/outlier scenario actually produces invalid points.
- `test_recording.py` — save/load/iterate `.jsonl` recordings, replay in original order, loop
  behavior, pre-connect error handling.

## Running

```bash
pytest perception/tests
pytest simulator/tests
```

or, from inside either package directory, simply `pytest` (each has its own `pyproject.toml`).

## Conventions going forward

- Unit tests live alongside the subsystem they test, e.g. `perception/tests/test_clustering.py`
  for `perception/src/clustering/`.
- Integration tests that exercise multiple stages together (e.g. simulator → preprocessing →
  clustering → objects) will be added once there are at least two stages to connect.
- Prefer deterministic seeds for anything using randomness (see `NoiseConfig.seed` /
  `common.config.Settings.lidar_random_seed`) so tests are reproducible. Exact-distance
  assertions should use zero noise (`NoiseConfig()` defaults or a scenario's `noise` override);
  statistical properties (noise/outliers/missing rate) should still be tested under a fixed seed
  rather than asserted probabilistically without one.
