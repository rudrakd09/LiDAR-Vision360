# Data Model

## Status

Phase 0/1 (Foundation), Phase 3 (Preprocessing), Phase 4 (Coordinate Transformation), Phase 5
(Clustering), Phase 6 (Geometric Classification), Phase 7 (Object Tracking), Phase 8
(2D Occupancy Grid Mapping), and Phase 9 (Collision/Risk Engine) complete. Models are defined in
`perception/src/models/` (pydantic v2). They are intentionally over-provisioned with `Optional`
fields for capabilities not implemented yet, so later phases only populate fields rather than
redesign the schema -- except where a phase's concept is genuinely orthogonal (see "Extensibility
rule" below), which Phase 8's
`OccupancyGrid` is the first case of.

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

## `DetectedObject` / `ShapeFeatures` (`models/objects.py`)

A single classified obstacle. Populated by `objects.GeometricClassifier` (Phase 6) from one
`ObstacleCluster` -- see [object-classification.md](object-classification.md) for the full
write-up.

| Field | Type | Populated by |
|---|---|---|
| `object_id` | `str` | Phase 6, from the source `ObstacleCluster.cluster_id` |
| `centroid` | `Point2D` | Phase 6, from `ObstacleCluster.centroid_x/_y` |
| `width`, `depth` | `float` | Phase 6 -- **explicitly reconciled, not copied unchanged**: `DetectedObject.width` (lateral/Y) = `ObstacleCluster.depth` (Y-extent); `DetectedObject.depth` (radial/X) = `ObstacleCluster.width` (X-extent). See `objects.classifier._to_detected_object`. |
| `distance` | `float` | Phase 6, from `ObstacleCluster.centroid_distance` |
| `bounding_box` | `BoundingBox \| None` | Phase 6, from `ObstacleCluster.min_x/max_x/min_y/max_y` |
| `point_count` | `int \| None` | Phase 6, from `ObstacleCluster.point_count` |
| `classification` | `ObjectClassification` | Phase 6 (default `UNKNOWN`) |
| `confidence` | `float` `[0,1]` | Phase 6 -- the winning rule's raw score, not a calibrated probability |
| `min_distance` | `float \| None` | Phase 6, from `ObstacleCluster.min_distance` |
| `angular_width` | `float \| None` | Phase 6, from `ObstacleCluster.angular_width` |
| `shape_features` | `ShapeFeatures \| None` | Phase 6 -- the full extracted feature set, for debugging/explainability |
| `classification_reason` | `list[str] \| None` | Phase 6 -- human-readable explanation |
| `velocity` | `Velocity2D \| None` | Phase 7 (tracking) -- `None` until `tracking_min_observations_for_velocity` real detections |
| `direction` | `float \| None` | Phase 7 |
| `track_id` | `str \| None` | Phase 7 -- persistent, unique for the `ObjectTracker` session's lifetime |
| `predicted_position` | `Point2D \| None` | Phase 7 -- Kalman one-nominal-scan-ahead estimate |
| `tracking_state` | `TrackingState \| None` | Phase 7 -- `TENTATIVE`/`CONFIRMED`/`COASTING`/`LOST` |
| `movement_state` | `MovementState \| None` | Phase 7 -- `STATIONARY`/`MOVING`/`UNKNOWN` |
| `track_age` | `int \| None` | Phase 7 -- scans since track creation |
| `track_hits` | `int \| None` | Phase 7 -- scans matched to a real detection |
| `track_misses` | `int \| None` | Phase 7 -- current consecutive-miss streak |
| `timestamp` | `float` | always |

`ObjectClassification` values: `wall`, `vehicle_like`, `pole_like`, `person_like`,
`large_obstacle`, `unknown`. (`pole_like` was named `pole` when this enum was first defined in
Phase 0, before Phase 6's classifier existed to specify it; renamed to match, confirmed safe via
grep -- nothing referenced the old name.) The classifier defaults to `unknown` when uncertain
rather than guessing.

`ShapeFeatures` (`objects.features.extract_features`'s output, also stored on
`DetectedObject.shape_features`): `point_count`, `width`, `depth`, `aspect_ratio`,
`min_distance`/`max_distance`/`centroid_distance`, `min_angle`/`max_angle`/`angular_width`,
`mean_distance`, `distance_variance`, `spatial_variance`, `point_density`, `linearity_score`,
`circularity_score`. See object-classification.md "Feature definitions" for what each one means.

This table originally (Phase 0) said these fields would be "populated by Phase 5 (clustering)."
In practice, Phase 5 -- like Phase 3 and 4 before it -- produced its own dedicated per-stage
model (`ObstacleCluster`/`ClusteredScan`) instead of populating `DetectedObject` directly; Phase 6
is what actually populates `DetectedObject`, mapping it from an `ObstacleCluster` plus its
extracted `ShapeFeatures` and classification result.

## `ClassifiedScan` (`models/classification.py`)

Output of `objects.GeometricClassifier.classify()` (Phase 6). `scan_id`/`sequence_number`/
`source_id`/`timestamp` (passthrough), `objects` (`list[DetectedObject]`, one per retained
cluster), `noise_points` (passthrough, unchanged, from the source `ClusteredScan` -- noise was
never a cluster, so there's nothing for this stage to classify), `object_count`, `noise_count`.

## `TrackedScan` (`models/tracking.py`)

Output of `tracking.ObjectTracker.update()` (Phase 7) -- see [tracking.md](tracking.md) for the
full write-up. Reuses `DetectedObject` for tracked objects too (no parallel per-object type; see
"Extensibility rule" below) -- `scan_id`/`sequence_number`/`source_id`/`timestamp` (passthrough
from the source `ClassifiedScan`), `objects` (`list[DetectedObject]`, one per live -- `TENTATIVE`,
`CONFIRMED`, or `COASTING` -- track this scan, `track_id`/`velocity`/`direction`/
`predicted_position`/`tracking_state`/`movement_state`/`track_age`/`track_hits`/`track_misses`
all populated), `noise_points` (passthrough, unchanged), `object_count`, `noise_count`,
`new_track_count`, `lost_track_count`, `coasting_track_count`.

`TrackingState` values: `tentative`, `confirmed`, `coasting`, `lost`.
`MovementState` values: `stationary`, `moving`, `unknown`.

## `CellState` / `VehiclePose` / `MapStatistics` / `OccupancyGrid` (`models/mapping.py`)

Output of `mapping.OccupancyGridMapper` (Phase 8) -- see [mapping.md](mapping.md) for the full
write-up. The first genuinely new, orthogonal model this project has introduced since Phase 0
(everything through Phase 7 either got its own per-stage *scan* model while reusing existing
per-point/per-object types, or -- Phase 7 -- reused `DetectedObject` outright); an occupancy grid
cell has no existing equivalent to extend.

- `CellState` (`IntEnum`, not this package's usual `str, Enum` -- so it stores directly in a
  numpy `int8` array): `UNKNOWN` (`0`), `FREE` (`1`), `OCCUPIED` (`2`).
- `VehiclePose { x, y, heading }` -- world-frame pose (meters, degrees), defaults to the identity
  pose `(0, 0, 0)`. Reusable by future phases needing a vehicle pose (collision, clearance,
  Unity) rather than each redefining one.
- `MapStatistics { total_cells, unknown_cells, free_cells, occupied_cells,
  occupied_percentage, free_percentage, unknown_percentage, width_cells, height_cells,
  resolution_m, width_m, height_m }`.
- `OccupancyGrid { width_cells, height_cells, resolution_m, origin_x_m, origin_y_m, max_range_m,
  timestamp, scan_count, cell_states, log_odds }` -- `cell_states`/`log_odds` are raw
  `numpy.ndarray` fields (`model_config = ConfigDict(arbitrary_types_allowed=True)`), a
  deliberate deviation from every earlier model's `list[SomePydanticModel]` pattern, justified by
  scale (a default grid is 160,000 cells) -- see the model's own docstring for the full reasoning.
  **Do not compare two `OccupancyGrid` instances with `==`** (numpy array equality inside
  pydantic's generated equality raises "truth value of an array is ambiguous") -- compare
  individual scalar fields, or the arrays via `np.array_equal(...)`, instead.

## `RiskLevel` / `VehicleState` / `CollisionRiskResult` / `CollisionAssessment` (`models/collision.py`)

Output of `collision.CollisionRiskEngine` (Phase 9) -- see [collision.md](collision.md) for the
full write-up. Reuses `VehiclePose` (Phase 8) and `Point2D`/`Velocity2D`/`ObjectClassification`
(Phase 0/6) rather than duplicating them -- the only genuinely new concepts are the risk
classification and the result containers themselves.

- `RiskLevel`: `safe`, `warning`, `critical`.
- `VehicleState { pose: VehiclePose, speed_mps }` -- `speed_mps` defaults to `0.0` (stationary).
- `CollisionRiskResult { track_id, classification, distance, relative_position, relative_velocity,
  relative_speed, in_projected_path, ttc, collision_predicted, predicted_collision_time,
  predicted_collision_position, risk_level, risk_score, reason, timestamp }` -- `distance` is
  recomputed fresh from `vehicle_state`, not reused from `DetectedObject.distance` (which always
  assumes the vehicle is at the world origin).
- `CollisionAssessment { scan_id, sequence_number, source_id, timestamp, results, object_count,
  overall_risk, most_critical_object }`.

## Supporting types

- `Point2D { x, y }` — vehicle-relative Cartesian point, meters.
- `Velocity2D { vx, vy }` — meters/second, with a `speed` property.
- `BoundingBox { min_x, max_x, min_y, max_y }` — vehicle-relative, meters.

## Extensibility rule

When adding fields for a new phase, prefer `Optional[...] = None`/sensible-default additions to
existing models over new parallel models, unless the concept is genuinely orthogonal. Phase 7
(tracking) followed the additive-fields half of this rule (extended `DetectedObject` rather than
introducing a parallel "TrackedObject" model); Phase 8 (mapping) is the first case of the other
half -- occupancy grid cells have no existing model to extend, so they got their own
(`models.mapping.OccupancyGrid`), exactly as anticipated when this rule was first written.
