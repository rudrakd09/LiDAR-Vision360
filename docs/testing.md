# Testing

## Status

Phase 0 (Foundation), Phase 2 (LiDAR Simulator), Phase 3 (Preprocessing), Phase 4 (Coordinate
Transformation), and Phase 5 (Obstacle Clustering) tests implemented, **287 tests total**. Full
coverage across the remaining phases (object detection/classification, tracking, Kalman
filtering, occupancy grid, collision detection, TTC, clearance, API, data validation) will be
built out as each phase lands, per **Phase 17**.

## Current test suites

Each subproject (`perception/`, `simulator/`) has its own `pyproject.toml`/`testpaths` and is run
independently -- running both `tests/` directories in a single `pytest` invocation from the repo
root currently collides on the shared `tests` package name (both have `tests/__init__.py`), so
run them as two separate invocations, as below.

### `perception/tests/` (152 tests)

- `test_models.py` — validation rules and construction for `LiDARPoint`, `CartesianPoint`,
  `DetectedObject`, `ScanFrame`.
- `test_config.py` — `Settings` defaults and `LIDAR_`-prefixed environment variable overrides.
- `test_datasources.py` — `LiDARDataSource` interface conformance, `SimulatedLiDARDataSource`
  scan generation (point count, angle/distance validity, sequence numbering, context-manager and
  streaming behavior), `SerialLiDARDataSource` stub raising `NotImplementedError` as expected.
- `test_preprocessing_windowing.py` — explicit 0/360 circular-boundary behavior of the shared
  windowing helper used by both outlier detection and the median filter.
- `test_preprocessing_validation.py` — per-reason validation classification (sensor-flagged,
  non-finite angle/distance, below/above range), including NaN/inf edge cases via
  `LiDARPoint.model_construct()`, and scan-level splitting (empty/completely-invalid/mixed).
- `test_preprocessing_outliers.py` — the spec's isolated-spike and legitimate-boundary examples,
  multiple isolated outliers, the documented narrow-feature limitation, 0/360 boundary cases,
  and window/threshold configuration effects.
- `test_preprocessing_denoise.py` — spec's noise-reduction example, edge preservation across a
  sharp step, window configuration, empty/degenerate scans.
- `test_preprocessing_temporal.py` — exact exponential-formula matching, per-angle isolation,
  reset behavior, and a numerically-verified steady-state lag bound for a constant-velocity ramp
  (with higher `alpha` shown to reduce that lag).
- `test_preprocessing_pipeline.py` — end-to-end `PreprocessedScan` structure, count/statistics
  consistency, angle-sort/preservation, determinism, and error handling (empty, completely
  invalid, partially invalid, malformed/NaN point, duplicate angles, sparse/missing angles,
  unusually small/large point counts).
- `test_coordinates_transformer.py` — cardinal-angle worked examples (0/90/180/270/45°), angle
  handling (negative angles, `360°`, `>360°`, original angle preserved unmodified), multiple
  points and ordering/correspondence preservation, empty/zero/near-zero/large-distance scans,
  invalid-measurement defense-in-depth (NaN, infinite, out-of-range angle via
  `LiDARPoint.model_construct()`), quality-statistics passthrough, determinism, and numerical
  accuracy against hand-computed trig plus a Pythagorean-identity check across 60 angles.
- `test_clustering_geometry.py` — `circular_angular_extent`'s 0/360 boundary handling: the
  spec's exact wrapping example, non-wrapping/single-angle/empty cases, and the ambiguous
  antipodal-points edge case.
- `test_clustering_dbscan.py` — the spec's synthetic Tests A-E (three close points → one
  cluster, two separated groups → two clusters, an isolated far point → noise not a cluster,
  points crossing the 0/360 boundary → one cluster, `min_samples` configuration), plus cluster
  model field correctness, empty/degenerate scans, free-space filtering, and determinism.
- `test_clustering_parameters.py` — too-small/too-large `eps` and `min_samples`, and a precise
  synthetic reproduction of the quasi-1D density insight that drove the final default parameters
  (see docs/clustering.md "Parameter selection").

### `simulator/tests/` (126 tests)

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
- `test_preprocessing_integration.py` — `perception.preprocessing` run against 7 required
  simulator scenarios: all preprocess without error; low-noise scans barely change point count;
  wall/vehicle-edge distances are preserved (not blurred); noise and outlier variance are
  measurably reduced within the wall's hit-cone; missing/outlier counts are accounted for;
  moving-obstacle scans show no added lag (temporal filtering off by default). Lives here rather
  than in `perception/tests/` since `perception.preprocessing` itself must stay
  simulator-independent -- see docs/preprocessing.md "Architecture".
- `test_coordinates_integration.py` — `perception.coordinates` chained after
  `perception.preprocessing`, run against 7 required simulator scenarios: all transform without
  error; Euclidean `sqrt(x^2+y^2)` reconstructs the original `distance` everywhere; the empty
  scenario's points sit on the max-range circle; the wall/vehicle-ahead scenarios' points sit at
  the expected `(x, y)`; the multi-obstacle scenario has points near each expected obstacle
  region; the narrow corridor's left/right walls sit at `y ≈ ±1.0`; moving-obstacle scenarios'
  nearest point stays geometrically consistent scan to scan, with the approaching obstacle's
  nearest-point `x` measurably decreasing over scans. Same "lives here, not in
  `perception/tests/`" reasoning as above -- see docs/coordinates.md "Architecture".
- `test_clustering_integration.py` — `perception.clustering` chained after preprocessing and
  coordinates, run against all 10 scenarios: empty → 0 clusters; wall/pole/vehicle-ahead → 1
  cluster each; multiple obstacles → several distinct clusters with the two poles staying
  compact and separate; narrow corridor → exactly 2 clusters at `y ≈ ±1.0`; moving/approaching
  obstacles → 1 coherent cluster across 5 scans each, with the approaching obstacle's centroid
  distance measurably shrinking; noisy LiDAR → no false-cluster explosion; missing/outliers →
  bounded fragmentation with a substantial main cluster surviving. Same "lives here" reasoning --
  see docs/clustering.md "Architecture".

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
  coordinates → clustering, and later → objects) live in whichever package already depends on
  the other(s), so no package's own test suite gains a dependency it doesn't otherwise need --
  see `simulator/tests/test_{preprocessing,coordinates,clustering}_integration.py`.
- Prefer deterministic seeds for anything using randomness (see `NoiseConfig.seed` /
  `common.config.Settings.lidar_random_seed`) so tests are reproducible. Exact-distance
  assertions should use zero noise (`NoiseConfig()` defaults or a scenario's `noise` override);
  statistical properties (noise/outliers/missing rate) should still be tested under a fixed seed
  rather than asserted probabilistically without one.
