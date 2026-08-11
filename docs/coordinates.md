# Coordinate Transformation

## Status

**Phase 4 — implemented.** Package: `perception/src/coordinates/`. Turns a
`models.preprocessing.PreprocessedScan` into a `models.coordinates.CartesianScan`. Has no
dependency on `simulator` or on any later pipeline stage (clustering, tracking, mapping,
collision, clearance) or on Unity/cloud -- same architecture boundary as `preprocessing`, see
docs/architecture.md.

## Polar coordinate representation

A raw/preprocessed LiDAR measurement is `(angle, distance)`:

- **angle**: degrees, `[0, 360)`, measured **counter-clockwise from the vehicle's forward axis**.
- **distance**: meters, `>= 0`.

This is `models.lidar.LiDARPoint` (also the point type inside `PreprocessedScan`), unchanged
since Phase 0.

## Cartesian coordinate representation

A Cartesian point adds `x`, `y` (meters) to the same `(angle, distance, timestamp)` measurement --
`models.lidar.CartesianPoint`, which has existed since Phase 0 specifically for this phase to
populate. No new/duplicate point type was introduced.

## Coordinate convention

**This phase did not introduce a new convention -- it implements the one already documented in
docs/data-model.md since Phase 0 and already used by the simulator's own ray-casting geometry
(`simulator.geometry.ray_direction`, Phase 2).** Inspected and confirmed consistent before writing
any code, per this phase's own instructions:

- LiDAR/vehicle at the origin `(0, 0)`.
- `0°` = forward = `+X`.
- `90°` = left = `+Y`.
- `180°` = backward = `-X`.
- `270°` = right = `-Y`.
- Angle increases **counter-clockwise**.

This is a standard right-handed 2D frame with the vehicle facing `+X`. It matches
`simulator`'s `ray_direction(angle_deg) = (cos, sin)` exactly, so a point's Cartesian position
computed here is identical to the position the simulator's ray-casting geometry already reasons
about internally -- one convention, used everywhere, confirmed rather than assumed.

## Mathematical equations

```text
theta = radians(angle)
x = distance * cos(theta)
y = distance * sin(theta)
```

Implemented in `coordinates.transformer.polar_to_cartesian(angles_deg, distances)` -- a pure,
vectorized NumPy function (`numpy.radians`, `numpy.cos`, `numpy.sin`) with no pydantic/model
dependency, so it's trivially unit-testable against hand-computed trig values and reusable
wherever raw polar arrays need converting.

Worked examples (all exact, verified in
`perception/tests/test_coordinates_transformer.py::TestCardinalAngles`):

| angle | distance | x | y |
|---|---|---|---|
| 0° | 5 m | 5.0 | 0.0 |
| 90° | 3 m | 0.0 | 3.0 |
| 180° | 4 m | -4.0 | 0.0 |
| 270° | 2 m | 0.0 | -2.0 |
| 45° | 1 m | 0.7071 | 0.7071 |

## Angle handling

Trigonometric functions are periodic, so `polar_to_cartesian` produces geometrically correct
output for **any** real angle value -- negative, `>= 360`, or exactly `360` -- with **no
normalization step**. `360°` and `0°` produce identical `(x, y)`; `450°` and `90°` produce
identical `(x, y)`; `-90°` and `270°` produce identical `(x, y)`. All verified directly in
`TestAngleHandling`.

**The original `angle` value is never modified.** `CoordinateTransformer` reads
`point.angle`/`point.distance` and writes a new `CartesianPoint` carrying the *same* `angle` and
`distance` alongside the newly computed `x`/`y` -- it does not rewrite, clamp, or renormalize the
stored angle, preserving the exact `angle -> distance -> (x, y)` correspondence end to end.

In practice, every point reaching this stage already comes from a normally-constructed
`LiDARPoint`, whose own pydantic constraint (`ge=0, lt=360`) makes negative/`>=360` angles
unconstructable through the pipeline's normal path -- so this robustness is defense in depth
(and directly testable via `LiDARPoint.model_construct()`, the same pattern
`preprocessing.validation` uses), not something the normal pipeline relies on day to day.

## Invalid data behavior

`preprocessing.Preprocessor` has already validated every point in a `PreprocessedScan` (Phase 3)
-- `CoordinateTransformer` does **not** re-run that validation. It does apply one narrow,
defense-in-depth safety net, per this phase's explicit requirement to handle "invalid
measurements reaching this stage unexpectedly" without crashing or emitting bad coordinates:

1. Skip any point whose `angle`/`distance` is non-finite (checked before conversion), or whose
   computed `x`/`y` is non-finite (checked after -- can't actually happen given finite inputs,
   but checked anyway).
2. Skip any point that fails to construct as a `CartesianPoint` (e.g. an angle somehow outside
   `[0, 360)` -- `CartesianPoint` inherits `LiDARPoint`'s own range constraint, so construction
   itself is the final check).
3. If anything was skipped, log one `WARNING` per scan naming the scan ID and count -- visible,
   not silent, per this phase's "do not silently hide serious data-quality problems" requirement.

A scan where every point is skipped this way returns a `CartesianScan` with an empty `points`
list rather than raising. See
`perception/tests/test_coordinates_transformer.py::TestInvalidMeasurementHandling`.

## Vectorization

The actual per-point arithmetic (`cos`, `sin`, multiply) is fully vectorized via NumPy --
`polar_to_cartesian` builds two `float64` arrays and does the whole scan's trig in a handful of
array operations, not a Python-level loop. Building the input arrays from the list of
`LiDARPoint`s, and building the output list of `CartesianPoint` pydantic objects, is an O(n)
Python loop -- unavoidable, since pydantic model construction can't itself be vectorized, and
deliberately not over-engineered further (see Performance below: this is not the bottleneck).

## Preserved scan structure

`CartesianScan.points` has the same length and order as the input `PreprocessedScan.points`
(minus any defensively-skipped points, per above) -- the transformer does not sort, reorder, or
deduplicate. Verified directly in `TestMultiplePointsAndOrdering`.

## Example usage

```python
from coordinates import CoordinateTransformer
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

preprocessor = Preprocessor()
transformer = CoordinateTransformer()
source = make_data_source("02_wall_in_front")

with source:
    raw_scan = source.read_scan()
    clean_scan = preprocessor.process(raw_scan)
    cartesian_scan = transformer.transform(clean_scan)

for p in cartesian_scan.points[:3]:
    print(p.angle, p.distance, p.x, p.y)
```

One-off convenience: `from coordinates import transform_scan; transform_scan(clean_scan)`.

Debug/benchmark tooling:

```bash
python scripts/visualize_cartesian.py --scenario 06_narrow_corridor --visualize
python scripts/benchmark_coordinates.py
```

## Performance

Measured with `scripts/benchmark_coordinates.py` (transformer time only, preprocessing excluded
-- see `scripts/benchmark_preprocessing.py` for that stage's own numbers), 300 scans/scenario,
360-point scenarios:

| Scenario | avg (ms) | max (ms) | min (ms) |
|---|---|---|---|
| `01_empty` | 1.04 | 10.13 | 0.72 |
| `02_wall_in_front` | 2.40 | 10.77 | 1.41 |
| `05_multiple_obstacles` | 2.26 | 14.22 | 1.43 |
| `09_noisy_lidar` | 2.33 | 15.39 | 1.41 |
| `10_missing_outliers` | 2.04 | 21.09 | 1.20 |

Overall average ~2.0ms/scan (~500 scans/second), comfortably over the 10+ scans/second real-time
target, with headroom to spare even at the occasional slower call (worst observed ~21ms, still
~48 scans/second). `01_empty` is fastest since there are no obstacles for preprocessing/here to
do extra per-point work on relative to the others; all remain well within budget.

## Limitations

- No 3D elevation -- this is a 2D transformation, consistent with the project's 2D-LiDAR scope
  throughout (`PROJECT_SPECIFICATION.md` "Important Sensor Limitation").
- The output `x`/`y` are rounded to 4 decimal places (matching the rounding already used
  elsewhere in the pipeline, e.g. `preprocessing.denoise`/`temporal`), not full `float64`
  precision -- negligible for a LiDAR with millimeter-scale real-world accuracy at best.
- `ScanFrame.cartesian_points` (a Phase-0-era placeholder field) is *not* populated by this
  stage -- `CoordinateTransformer` produces a dedicated `CartesianScan` instead, following the
  same per-stage-model pattern `PreprocessedScan` established in Phase 3. See
  `models/scan.py`'s docstring and docs/data-model.md.
