# Data Model

## Status

Phase 0/1 (Foundation), Phase 3 (Preprocessing), Phase 4 (Coordinate Transformation), and Phase 5
(Clustering) complete. Models are defined in `perception/src/models/` (pydantic v2). They are
intentionally over-provisioned with `Optional` fields for capabilities not implemented yet, so
later phases only populate fields rather than redesign the schema.

## Coordinate convention

- **angle**: degrees, range `[0, 360)`, measured **counter-clockwise from the vehicle's forward
  axis**.
- **distance**: meters, `>= 0`. `0` represents "no return" rather than "object at the origin".
- **Cartesian conversion** (implemented by `perception/src/coordinates/`, Phase 4 -- see
  [coordinates.md](coordinates.md) for the full write-up):

  ```
  x = distance * cos(radians(angle))
  y = distance * sin(radians(angle))
  ```

  `x` is thus the vehicle-forward axis and `y` the vehicle-left axis in a standard
  right-handed 2D frame, with the vehicle origin at `(0, 0)`. `0°`=+X, `90°`=+Y, `180°`=-X,
  `270°`=-Y. This is the same convention `simulator.geometry.ray_direction` already used
  (confirmed consistent, not changed, when Phase 4 was implemented).

- **timestamp**: Unix epoch seconds, `float`.

## `LiDARPoint` (`models/lidar.py`)

A single raw polar measurement. This is the format every `LiDARDataSource` must emit.

| Field | Type | Notes |
|---|---|---|
| `angle` | `float` | `[0, 360)` degrees |
| `distance` | `float` | meters, `>= 0` |
| `timestamp` | `float` | epoch seconds |
| `valid` | `bool` | default `True`; `False` means the *data source* already flagged this measurement invalid (e.g. a dropout). Preprocessing (Phase 3) reads this as one validation input; retained points in a `PreprocessedScan` are always `valid=True`. |
| `intensity` | `float \| None` | optional return-signal intensity |

## `CartesianPoint` (`models/lidar.py`)

Subclasses `LiDARPoint`, adding `x` and `y` (meters). Produced by `coordinates.CoordinateTransformer`
(Phase 4); retains the original polar fields (`angle`, `distance` -- unmodified) so nothing
downstream needs to re-derive them.

## `ScanFrame` (`models/scan.py`)

One complete 360° sweep.

| Field | Type | Populated by |
|---|---|---|
| `scan_id` | `str` | data source |
| `sequence_number` | `int` | data source (monotonic per-source counter) |
| `source_id` | `str` | data source (e.g. `"simulated"`, future `"stm32-uart"`) |
| `timestamp` | `float` | data source |
| `points` | `list[LiDARPoint]` | data source |
| `cartesian_points` | `list[CartesianPoint] \| None` | Reserved, unused placeholder -- see field/class docstring in `scan.py`. The actual pipeline produces a dedicated `CartesianScan` instead (below). |
| `objects` | `list[DetectedObject] \| None` | Reserved, unused placeholder, same reasoning. |

## `PreprocessedScan` / `ScanQualityStatistics` (`models/preprocessing.py`)

Output of `preprocessing.Preprocessor.process()` (Phase 3) -- see
[preprocessing.md](preprocessing.md) for the full pipeline write-up. Reuses `LiDARPoint` for
retained points rather than a parallel type, since a clean measurement is still just
`(angle, distance, timestamp)`.

| Field | Type | Notes |
|---|---|---|
| `scan_id`, `sequence_number`, `source_id`, `timestamp` | passthrough from the source `ScanFrame` |
| `points` | `list[LiDARPoint]` | final retained measurements only, angle-sorted, `valid=True` |
| `total_count` | `int` | measurements in the source scan |
| `valid_count` | `int` | passed validation (may still include later-removed outliers) |
| `invalid_count` | `int` | `total_count - valid_count` |
| `outlier_count` | `int` | of the valid ones, how many were removed as local outliers |
| `quality_statistics` | `ScanQualityStatistics` | see below |

## `CartesianScan` (`models/coordinates.py`)

Output of `coordinates.CoordinateTransformer.transform()` (Phase 4) -- see
[coordinates.md](coordinates.md) for the full pipeline write-up. Reuses `CartesianPoint` for
points rather than a parallel type.

| Field | Type | Notes |
|---|---|---|
| `scan_id`, `sequence_number`, `source_id`, `timestamp` | passthrough from the source `PreprocessedScan` |
| `points` | `list[CartesianPoint]` | same order and (minus any defensively-dropped points) count as the input; `angle`/`distance` unmodified, `x`/`y` newly computed |
| `quality_statistics` | `ScanQualityStatistics \| None` | carried through unchanged from the source `PreprocessedScan` |

`len(points) == valid_count - outlier_count`.

`ScanQualityStatistics`: `valid_percentage` / `invalid_percentage` / `outlier_percentage`
(relative to `total_count`, `0.0` for an empty scan) and `mean_distance` / `median_distance` /
`minimum_distance` / `maximum_distance` (computed over `points`; `None`, not `0.0`, when there
are none). Consumed later by sensor-health monitoring and the cloud dashboard (Phases 12/14/24).

## `ObstacleCluster` / `ClusteredScan` (`models/clustering.py`)

Output of `clustering.DBSCANClusterer.cluster()` (Phase 5) -- see [clustering.md](clustering.md)
for the full pipeline write-up. Reuses `CartesianPoint` for member points.

| Field | Type | Notes |
|---|---|---|
| `cluster_id` | `int` | Stable within this scan only (`0, 1, 2, ...`); not a persistent cross-scan track ID (Phase 7). |
| `points` | `list[CartesianPoint]` | Member points, original scan order. |
| `point_count` | `int` | |
| `centroid_x`, `centroid_y` | `float` | `mean(x)`, `mean(y)` of member points. |
| `min_x`, `max_x`, `min_y`, `max_y` | `float` | Axis-aligned bounding box. |
| `width` | `float` | `max_x - min_x` -- **the vehicle-forward/X extent.** See the axis-convention note below. |
| `depth` | `float` | `max_y - min_y` -- **the vehicle-lateral/Y extent.** |
| `min_distance`, `max_distance` | `float` | Extremes of member points' own polar `distance`. |
| `centroid_distance` | `float` | Distance from the LiDAR origin to `(centroid_x, centroid_y)` -- computed fresh, not a per-point value. |
| `min_angle`, `max_angle`, `angular_width` | `float` | Shortest arc containing every member angle, circular-boundary-aware -- see `clustering.geometry.circular_angular_extent`. |
| `timestamp` | `float` | |

**Axis-convention note:** `width`/`depth` here are the *opposite* mapping from
`DetectedObject.width`/`.depth` (lateral/radial, Phase 0) -- a deliberate, documented
inconsistency between the two phases' specs. See [clustering.md](clustering.md) "Cluster
representation" for the full explanation and the Phase 6 recommendation.

`ClusteredScan`: `scan_id`/`sequence_number`/`source_id`/`timestamp` (passthrough), `clusters`,
`noise_points` (everything not in a cluster: DBSCAN noise, sub-minimum clusters, and
free-space/no-return points -- see clustering.md), `cluster_count`, `noise_count`,
`total_points`, `clustered_points`, `noise_percentage`, `average_cluster_size`,
`largest_cluster_size`, `smallest_cluster_size`.

## `DetectedObject` (`models/objects.py`)

A single tracked/detected obstacle.

| Field | Type | Populated by |
|---|---|---|
| `object_id` | `str` | Phase 6 (classification), from an `ObstacleCluster`'s `cluster_id` |
| `centroid` | `Point2D` | Phase 6, from `ObstacleCluster.centroid_x/_y` |
| `width`, `depth` | `float` | Phase 6 -- **note the axis convention differs from `ObstacleCluster.width/.depth`, see below; do not copy unchanged** |
| `distance` | `float` | Phase 6, from `ObstacleCluster.centroid_distance` |
| `bounding_box` | `BoundingBox \| None` | Phase 6, from `ObstacleCluster.min_x/max_x/min_y/max_y` |
| `point_count` | `int \| None` | Phase 6, from `ObstacleCluster.point_count` |
| `classification` | `ObjectClassification` | Phase 6 (default `UNKNOWN`) |
| `confidence` | `float` `[0,1]` | Phase 6 (default `0.0`) |
| `velocity` | `Velocity2D \| None` | Phase 7 (tracking) |
| `direction` | `float \| None` | Phase 7 |
| `track_id` | `str \| None` | Phase 7 |
| `timestamp` | `float` | always |

`ObjectClassification` values: `wall`, `vehicle_like`, `pole`, `person_like`, `large_obstacle`,
`unknown`. The classifier (Phase 6) must default to `unknown` when uncertain rather than guess.

This table originally (Phase 0) said these fields would be "populated by Phase 5 (clustering)."
In practice, Phase 5 -- like Phase 3 and 4 before it -- produces its own dedicated per-stage
model (`ObstacleCluster`/`ClusteredScan`, see below) rather than populating `DetectedObject`
directly; `DetectedObject` will be populated *from* an `ObstacleCluster` once Phase 6
(classification) exists to decide what each cluster is.

## Supporting types

- `Point2D { x, y }` — vehicle-relative Cartesian point, meters.
- `Velocity2D { vx, vy }` — meters/second, with a `speed` property.
- `BoundingBox { min_x, max_x, min_y, max_y }` — vehicle-relative, meters.

## Extensibility rule

When adding fields for a new phase, prefer `Optional[...] = None`/sensible-default additions to
existing models over new parallel models, unless the concept is genuinely orthogonal (e.g.
occupancy grid cells, which will get their own model in Phase 8).
