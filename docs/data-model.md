# Data Model

## Status

Phase 0/1 (Foundation) complete. Models are defined in `perception/src/models/` (pydantic v2).
They are intentionally over-provisioned with `Optional` fields for capabilities not implemented
yet, so later phases only populate fields rather than redesign the schema.

## Coordinate convention

- **angle**: degrees, range `[0, 360)`, measured **counter-clockwise from the vehicle's forward
  axis**.
- **distance**: meters, `>= 0`. `0` represents "no return" rather than "object at the origin".
- **Cartesian conversion** (Phase 4 will implement this in `perception/src/coordinates/`):

  ```
  x = distance * cos(radians(angle))
  y = distance * sin(radians(angle))
  ```

  `x` is thus the vehicle-forward axis and `y` the vehicle-left axis in a standard
  right-handed 2D frame, with the vehicle origin at `(0, 0)`.

- **timestamp**: Unix epoch seconds, `float`.

## `LiDARPoint` (`models/lidar.py`)

A single raw polar measurement. This is the format every `LiDARDataSource` must emit.

| Field | Type | Notes |
|---|---|---|
| `angle` | `float` | `[0, 360)` degrees |
| `distance` | `float` | meters, `>= 0` |
| `timestamp` | `float` | epoch seconds |
| `valid` | `bool` | default `True`; set `False` by preprocessing (Phase 3) to flag without dropping |
| `intensity` | `float \| None` | optional return-signal intensity |

## `CartesianPoint` (`models/lidar.py`)

Subclasses `LiDARPoint`, adding `x` and `y` (meters). Produced by Phase 4's coordinate
conversion; retains the original polar fields so nothing downstream needs to re-derive them.

## `ScanFrame` (`models/scan.py`)

One complete 360° sweep.

| Field | Type | Populated by |
|---|---|---|
| `scan_id` | `str` | data source |
| `sequence_number` | `int` | data source (monotonic per-source counter) |
| `source_id` | `str` | data source (e.g. `"simulated"`, future `"stm32-uart"`) |
| `timestamp` | `float` | data source |
| `points` | `list[LiDARPoint]` | data source |
| `cartesian_points` | `list[CartesianPoint] \| None` | Phase 4 (coordinates) |
| `objects` | `list[DetectedObject] \| None` | Phase 5-6 (clustering/classification) |

## `DetectedObject` (`models/objects.py`)

A single tracked/detected obstacle.

| Field | Type | Populated by |
|---|---|---|
| `object_id` | `str` | Phase 5 (clustering) |
| `centroid` | `Point2D` | Phase 5 |
| `width`, `depth` | `float` | Phase 5 |
| `distance` | `float` | Phase 5 |
| `bounding_box` | `BoundingBox \| None` | Phase 5 |
| `point_count` | `int \| None` | Phase 5 |
| `classification` | `ObjectClassification` | Phase 6 (default `UNKNOWN`) |
| `confidence` | `float` `[0,1]` | Phase 6 (default `0.0`) |
| `velocity` | `Velocity2D \| None` | Phase 7 (tracking) |
| `direction` | `float \| None` | Phase 7 |
| `track_id` | `str \| None` | Phase 7 |
| `timestamp` | `float` | always |

`ObjectClassification` values: `wall`, `vehicle_like`, `pole`, `person_like`, `large_obstacle`,
`unknown`. The classifier (Phase 6) must default to `unknown` when uncertain rather than guess.

## Supporting types

- `Point2D { x, y }` — vehicle-relative Cartesian point, meters.
- `Velocity2D { vx, vy }` — meters/second, with a `speed` property.
- `BoundingBox { min_x, max_x, min_y, max_y }` — vehicle-relative, meters.

## Extensibility rule

When adding fields for a new phase, prefer `Optional[...] = None`/sensible-default additions to
existing models over new parallel models, unless the concept is genuinely orthogonal (e.g.
occupancy grid cells, which will get their own model in Phase 8).
