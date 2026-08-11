# Testing

## Status

Phase 0 (Foundation) tests implemented. Full coverage across all phases (coordinate conversion,
filtering, clustering, object detection, classification, tracking, Kalman filtering, occupancy
grid, collision detection, TTC, clearance, API, data validation, plus integration tests using
simulated environments) will be built out as each phase lands, per **Phase 17**.

## Current test suite

`perception/tests/`, run with `pytest`:

- `test_models.py` — validation rules and construction for `LiDARPoint`, `CartesianPoint`,
  `DetectedObject`, `ScanFrame`.
- `test_config.py` — `Settings` defaults and `LIDAR_`-prefixed environment variable overrides.
- `test_datasources.py` — `LiDARDataSource` interface conformance, `SimulatedLiDARDataSource`
  scan generation (point count, angle/distance validity, sequence numbering, context-manager and
  streaming behavior), `SerialLiDARDataSource` stub raising `NotImplementedError` as expected.

## Running

```bash
pytest perception/tests
```

or, from inside `perception/`, simply `pytest` (configured via `perception/pyproject.toml`).

## Conventions going forward

- Unit tests live alongside the subsystem they test, e.g. `perception/tests/test_clustering.py`
  for `perception/src/clustering/`.
- Integration tests that exercise multiple stages together (e.g. simulator → preprocessing →
  clustering → objects) will be added once there are at least two stages to connect.
- Prefer deterministic seeds for anything using randomness (see
  `SimulatedLiDARDataSource(seed=...)`) so tests are reproducible.
