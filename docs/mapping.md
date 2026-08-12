# 2D Occupancy Grid Mapping

## Status

**Implemented (Phase 8)**, in `perception/src/mapping/`. Builds on Phase 0 (Foundation),
Phase 3 (Preprocessing), Phase 4 (Coordinate Transformation), Phase 5 (Obstacle Clustering),
Phase 6 (Geometric Object Classification), and Phase 7 (Object Tracking) -- all complete and
unmodified by this phase (mapping consumes `CartesianScan` directly; it has no dependency on
clustering/classification/tracking, see "Architecture" below). Collision engine through hardware
integration (Phases 9-10, 16) are not implemented yet.

**This is a 2D LiDAR occupancy map, not a true 3D map.** The underlying sensor is a 2D 360°
LiDAR (one horizontal plane of range measurements); every grid cell represents a small patch of
that plane around the vehicle, not a voxel of 3D space. Unity may later render a 3D digital-twin
*representation* of this 2D map (Phase 11), but no 3D reconstruction happens anywhere in this
phase -- see docs/architecture.md "Sensor limitation."

## Why occupancy mapping is required

Every earlier stage reports only what a *single current scan* looks like: which objects were
just detected, tracked, moving. None of them remember what the *empty space* around the vehicle
looks like, or retain belief about a part of the environment the current scan's rays didn't
happen to sweep this instant. An occupancy grid is the first stage that builds and *persists* a
map of the whole surrounding area -- FREE, OCCUPIED, or UNKNOWN -- accumulated across many scans,
which is exactly what a future collision/clearance engine (Phases 9-10) needs to reason about
"is there room here" rather than just "what did I just see."

## Architecture

```
CartesianScan
     |
for each point: rotate/translate by vehicle_pose, cap no-return rays at mapping_max_range_m
     |
Ray Traversal            (mapping.raytrace.bresenham_line, on quantized grid cells)
     |
Log-Odds Update          (free cells along the ray, occupied/free cell at the ray's end)
     |
OccupancyGrid             (persists across calls -- NOT recreated each scan)
```

`perception/src/mapping/`:

| Module | Responsibility |
|---|---|
| `coordinate_transform.py` | Pure `world_to_grid`/`grid_to_world` conversion, no model dependency. |
| `raytrace.py` | `bresenham_line()` -- which grid cells a straight line between two cells passes through. |
| `grid.py` | `OccupancyGridMapper` -- the only public entry point; orchestrates predict/update/lifecycle once per scan. |
| `statistics.py` | `compute_map_statistics()` -- aggregate FREE/OCCUPIED/UNKNOWN counts and percentages. |
| `metrics.py` | Generic ground-truth evaluation (precision/recall/IoU), no `simulator` dependency. |
| `visualize.py` | Debug-only matplotlib plot, with optional raw-point/cluster/tracked-object overlays. |

This module has no dependency on `simulator` or any later pipeline stage (collision, clearance,
Unity, cloud, database, STM32) -- the same architecture boundary every prior phase established
(see docs/architecture.md). Like `tracking.ObjectTracker` (Phase 7) and unlike every stateless
`*Clusterer`/`*Classifier`/`*Transformer` before it, `OccupancyGridMapper` is inherently
**stateful** -- the map is exactly the thing that must persist scan-to-scan. One instance must be
constructed once and reused across every scan in a stream.

**Mapping consumes `CartesianScan` directly, not `ClassifiedScan`/`TrackedScan`.** Unlike
clustering → classification → tracking (each consuming the previous stage's typed output), the
occupancy grid needs raw per-point ray geometry (every measurement, hit or no-return), not
already-clustered/classified/tracked objects -- an occupancy grid's FREE cells come from *every*
ray's empty middle, most of which never becomes part of any cluster at all. `mapping` therefore
sits as a parallel consumer of Phase 4's output, not a strictly-linear next stage after Phase 7 in
the dependency graph, even though it is documented and numbered as "Phase 8" and runs after
tracking in the top-level pipeline scripts for convenience.

## Grid representation

A dense 2D array of log-odds occupancy evidence (`float64`), one cell per `resolution_m` x
`resolution_m` patch of the world. `CellState` (`UNKNOWN`/`FREE`/`OCCUPIED`) is derived from the
log-odds value on demand, not stored redundantly.

**Why log-odds, not raw probability:** log-odds updates are simple *additions* (`l += update`)
rather than probability's multiplicative Bayesian update, numerically stable (no risk of a
probability drifting outside `[0, 1]` from repeated float operations), and symmetric around `0`
(negative = leaning free, positive = leaning occupied, `0` = perfectly unknown) -- this is the
standard representation for occupancy-grid mapping (Thrun et al., *Probabilistic Robotics*) and
was chosen for exactly those well-established reasons, not picked arbitrarily.

**Why a `numpy.ndarray`, not a list of pydantic cell objects** (a deliberate deviation from every
earlier stage's `list[SomePydanticModel]` output pattern): a default grid is 400x400 = 160,000
cells; wrapping each individually would be roughly three orders of magnitude more memory and far
slower to update than one contiguous array, and the per-scan update genuinely needs vectorized
numpy operations (`get_grid()`'s log-odds → `CellState` discretization is a single vectorized
pass over the whole grid) to stay fast -- see "Performance" below. See
`models.mapping.OccupancyGrid`'s own docstring for the full reasoning.

## Coordinate system

Unchanged from Phase 4 (docs/coordinates.md): `+x` forward, `+y` left, meters, vehicle/LiDAR at
the world origin under the default identity pose.

- **World origin**: the vehicle's position at pose `(0, 0, 0)` -- the same convention every
  earlier stage already uses.
- **Grid origin** (`origin_x_m`, `origin_y_m`): the world-frame coordinate of grid cell
  `(row=0, col=0)`'s minimum corner. Defaults to `(-width_m/2, -height_m/2)`, centering the
  vehicle on the grid; overridable via constructor argument (this phase's "map origin" config
  requirement).
- **Cell indexing**: `grid_array[row][col]` -- `row` increases with world `y`, `col` increases
  with world `x`. This matches `numpy`/`matplotlib.imshow`'s own `[row][col]` <-> `(y, x)`
  convention for 2D rasters, so the grid array plots directly without a transpose.
- **Coordinate-to-grid**: `col = floor((x - origin_x_m) / resolution_m)`,
  `row = floor((y - origin_y_m) / resolution_m)`; `None` if outside `[0, width_cells) x
  [0, height_cells)`.
- **Grid-to-coordinate**: the *center* of a cell, `x = origin_x_m + (col + 0.5) * resolution_m`,
  `y = origin_y_m + (row + 0.5) * resolution_m`.

(`perception/src/mapping/coordinate_transform.py`, directly unit-tested in
`perception/tests/test_mapping_coordinate_transform.py`.)

## Ray traversal

**Algorithm: Bresenham's line algorithm**, applied to the already-quantized `(row, col)` integer
endpoints (the origin cell and the hit/no-return cell), not Amanatides & Woo's continuous-space
voxel traversal. Chosen because:

- Pure integer arithmetic -- no floating-point drift over a long ray, no extra boundary-crossing
  bookkeeping.
- One well-known algorithm handles every case (horizontal, vertical, diagonal, any octant,
  negative-direction, single-cell) by construction rather than special-cased branches -- see
  `perception/tests/test_mapping_raytrace.py` for the exact cases exercised.
- At this project's default resolution (0.1m/cell), the difference between "exact continuous
  line" (Amanatides & Woo) and "nearest integer-quantized line" (Bresenham on already-quantized
  endpoints) is at most a fraction of a cell -- immaterial for building a FREE/OCCUPIED/UNKNOWN
  belief grid.

Amanatides & Woo remains a documented future upgrade if a later phase needs precise fractional
ray-cell overlap (e.g. weighting a cell's update by exactly how much of the ray's length crossed
it).

## Ray-based mapping / occupancy update model

For each valid measurement in a `CartesianScan`:

1. Determine if it's a **real hit** (`distance < mapping_max_range_m - mapping_no_return_margin_m`)
   or a **no-return** (at/near the sensor's max range -- reuses the exact same "how close to max
   range counts as no signal" concept `clustering_max_range_margin_m` already established, see
   docs/clustering.md "Free-space filtering," given its own knob here since mapping and
   clustering are independent consumers).
2. Compute the ray's world-frame endpoint: a real hit's own `(distance, angle)`; a no-return
   ray's endpoint capped at exactly `mapping_max_range_m` along the same direction (not the raw,
   possibly-noisy reported distance).
3. Traverse the grid cells from the sensor origin to that endpoint (`bresenham_line`).
4. **Every cell except the last** gets a **FREE** log-odds update (`-mapping_free_update`).
5. **The last cell** gets an **OCCUPIED** update (`+mapping_occupied_update`) if this was a real
   hit, or a **FREE** update if it was a no-return -- a no-return ray must never mark its own
   capped endpoint as an obstacle; it only means "nothing detected within range," not "something
   is exactly at the max-range boundary."

Log-odds are clamped to `[mapping_min_log_odds, mapping_max_log_odds]` after every update,
preventing unbounded growth from thousands of repeated observations.

**Update magnitudes, derived from an inverse sensor model, not picked arbitrarily:**

| Setting | Default | Implied P(occupied \| evidence) |
|---|---|---|
| `mapping_occupied_update` | `+0.85` | `~0.70` given a direct hit |
| `mapping_free_update` | `-0.4` | `~0.40` given a ray passing through unobstructed |

The free update's smaller magnitude is deliberate: occupied evidence is intentionally "stickier"
than free evidence, so a single false pass-through does not erase a real obstacle's accumulated
belief as fast as a single real hit establishes it -- a safety-favoring asymmetry, not an
oversight.

## Cell states

`CellState` is derived from log-odds on demand:

- `log_odds <= mapping_free_threshold` (`-0.3`) → **FREE**
- `log_odds >= mapping_occupied_threshold` (`+0.7`) → **OCCUPIED**
- otherwise → **UNKNOWN**

Both thresholds have smaller magnitude than their corresponding *single-observation* update
(`|free_threshold|=0.3 < |free_update|=0.4`; `|occupied_threshold|=0.7 < |occupied_update|=0.85`),
so **one** real observation already crosses into FREE/OCCUPIED territory -- the discretized state
reflects current best belief immediately, while the underlying log-odds value keeps accumulating
(and resisting later contradiction) with repeated observations. A cell's initial, never-observed
log-odds (`0.0`) correctly falls in neither band, i.e. UNKNOWN.

## Vehicle position

`OccupancyGridMapper.update(scan, vehicle_pose=None)` accepts a `models.mapping.VehiclePose`
(`x`, `y` meters, `heading` degrees -- same convention as `LiDARPoint.angle`). Phase 8's own
explicit scope is the **identity pose** (`x=0, y=0, heading=0`, i.e. world frame == vehicle
frame) -- localization/SLAM is a future phase. The mapper genuinely *applies* whatever pose it is
given (rotates every point by `heading`, translates by `x`/`y`) rather than silently ignoring a
non-identity one, so a future phase only needs to start supplying a real, moving pose -- not
redesign this API (`perception/tests/test_mapping_grid.py::TestVehiclePoseTransform` exercises
this directly with a non-identity pose today, even though nothing in this phase calls `update()`
that way yet).

## Map updates over time / API

```python
mapper = OccupancyGridMapper(settings)   # reads defaults from common.config.Settings
mapper.reset()                            # (or mapper.clear() -- alias)
grid = mapper.update(cartesian_scan, vehicle_pose)   # persists internally; returns a snapshot
grid = mapper.get_grid()                  # (or mapper.get_map() -- alias) -- re-fetch without updating
stats = mapper.get_statistics()
```

`reset()`/`clear()` are the same method (aliased) -- the spec names both, and there is no
behavioral difference between "reset" and "clear" for this map, so one implementation covers
both names rather than duplicating logic. Likewise `get_grid()`/`get_map()`. Every `get_grid()`
call returns an independent `.copy()`'d snapshot -- mutating a returned `OccupancyGrid` can never
corrupt the mapper's live internal state.

## Decay / dynamic environment

Optional (`mapping_decay_enabled`, default `False`): at the start of each `update()` call, before
that scan's own ray updates are applied, every cell's log-odds is pulled toward `0` (UNKNOWN) by
`log_odds *= (1 - mapping_decay_rate)` (default rate `0.02`).

**Why this mostly matters for cells that stop being observed, not for a genuinely-stationary
obstacle:** with a 360° LiDAR at a fixed vehicle position (Phase 8's scope), a real stationary
obstacle is reinforced by fresh occupied evidence *every single scan*, which -- at the default
rate -- overwhelms the small decay applied that same cycle (e.g. `0.02 * 0.85 ≈ 0.017` decayed vs.
`+0.85` reinforced). Decay mainly helps: (a) a cell that genuinely stops being observed (e.g. an
obstacle that moved away, now correctly getting explicit *contradicting* FREE evidence from rays
passing through where it used to be -- the primary, faster adaptation mechanism, not decay itself;
see "Scenario results" below), (b) transient noise cells that get an occasional stray hit then
nothing, and (c) becomes considerably more relevant once a future phase supplies a genuinely
moving vehicle pose and cells fall in and out of sensor range. Defaults **off** -- an explicit
opt-in, per this phase's own spec warning ("do not make the map decay so aggressively that
stationary obstacles disappear").

## Map boundaries

A measurement (or the vehicle pose itself) landing outside the configured grid is **ignored, not
clipped or reprojected** -- one of this phase's explicitly sanctioned behaviors. Never crashes;
every such measurement increments `OccupancyGridMapper.out_of_bounds_count` (cumulative since the
last `reset()`) for diagnostics, and every *other*, in-bounds measurement in the same scan is
still processed normally. With the default grid (40m x 40m, vehicle centered) comfortably
exceeding the default sensor max range (12m) in every direction, this path is essentially never
hit under default configuration -- exercised directly in tests via a deliberately small grid
(`perception/tests/test_mapping_grid.py::Test09MapBoundaries`).

## Map statistics

`OccupancyGridMapper.get_statistics()` → `models.mapping.MapStatistics`: `total_cells`,
`unknown_cells`, `free_cells`, `occupied_cells`, `occupied_percentage`, `free_percentage`,
`unknown_percentage`, `width_cells`, `height_cells`, `resolution_m`, `width_m`, `height_m`.

## Ground-truth evaluation

`mapping.metrics.occupancy_accuracy_metrics()` (no `simulator` dependency, mirroring
`objects.metrics`/`tracking.metrics`'s own separation) computes precision/recall/IoU and
false-occupied/false-free cell counts for the binary "is this cell OCCUPIED" question, given a
predicted and a ground-truth `CellState` array of the same shape. **A cell `UNKNOWN` on either
side is excluded from every count** -- there is no ground truth to check against a cell the
rasterization itself doesn't cover, and a predicted-`UNKNOWN` cell is an honest "not yet
observed," not a wrong guess.

Ground truth itself (rasterizing a scenario's *known, exact* obstacle geometry into a comparable
grid) is scenario-specific knowledge that belongs in `scripts/evaluate_mapping.py`, which already
depends on both `simulator` and `perception` (mirroring `scripts/evaluate_classification.py`/
`evaluate_tracking.py`'s established separation) -- see "Ground Truth" results below.

**Methodology and its inherent limit:** a grid cell's ground truth is OCCUPIED if its center
falls within one grid resolution of a known obstacle's exact geometric surface (pole: distance to
its circle boundary; rectangle: distance to its nearest edge; wall: distance to its line segment),
FREE if within the mapper's max range but not near a surface, UNKNOWN beyond max range.
**Recall is inherently capped well below 1.0**, and this is expected, not a mapper defect: this
rasterization marks a *complete* ring/perimeter around each obstacle as "should be occupied,"
while a 2D LiDAR from one fixed vehicle position can only ever observe the *near-facing* side --
the far side is geometrically unobservable, exactly the same "2D LiDAR sees a surface, not a
silhouette's entire outline" limitation docs/tracking.md already documented for
`08_approaching_obstacle`'s position-error footnote. Precision is the more informative number for
this reason: it should be (and is) high, since anything the mapper *does* claim OCCUPIED should
genuinely be at/near a real surface. **This evaluation does not claim, and should not be read as
claiming, perfect mapping accuracy.**

## Test visualization

`mapping.visualize.plot_occupancy_grid()` (matplotlib, optional `viz` extra): the FREE (white) /
OCCUPIED (black) / UNKNOWN (gray) raster, vehicle/LiDAR position, coordinate axes, and an explicit
map-boundary outline; with optional overlays of raw `CartesianPoint`s, `ObstacleCluster`s, and
tracked `DetectedObject`s (track-ID-labeled) for cross-checking the map against whatever produced
it. Debugging/algorithm-validation only -- this is what will later map onto the Unity digital
twin's environment representation (Phase 11), not a replacement for it.

```
python scripts/visualize_mapping.py --scenario 02_wall_in_front --scans 10 --explain
python scripts/visualize_mapping.py --scenario 05_multiple_obstacles --scans 10 --visualize --overlay
```

## Scenario results

From `scripts/visualize_mapping.py`/manual runs (5-10 scans each, default settings):

| Scenario | Occupied cells | Behavior |
|---|---|---|
| `01_empty` | `0` | Entirely FREE/UNKNOWN, as required -- no obstacle anywhere. |
| `02_wall_in_front` | `~199` (5 scans) | One long, continuous occupied structure. |
| `03_pole_left` | `3` | A small, localized occupied region -- correctly nowhere near wall-scale. |
| `04_vehicle_ahead` | `18` | A compact rectangular occupied region -- larger than the pole, far smaller than the wall. |
| `05_multiple_obstacles` | `140` (10 scans), **5 distinct connected regions** | Separately identifiable occupied clusters, confirmed via connected-component labeling in `simulator/tests/test_mapping_integration.py`. |
| `06_narrow_corridor` | `192`, two separate wall boundaries | Free space (the corridor itself, vehicle's own position confirmed FREE) between two distinct occupied boundaries. |
| `07_moving_crossing` | shifts scan-to-scan | Occupied-region centroid moves a physically-consistent distance (>0.5m over ~1.2s at the pole's known 1.5 m/s) between an early and a late scan -- the map adapts, old positions are not left as a stale trail (see "Decay" above for *why* -- contradicting FREE evidence, not decay, is the primary mechanism). |
| `08_approaching_obstacle` | grows over time | Maps without error across 15 scans; occupied cell count present and non-zero throughout as the vehicle-like obstacle closes distance. |
| `09_noisy_lidar` | present, noisier | Still maps the wall's main structure; precision/recall both degrade vs. the clean scenario (see "Ground Truth" below) but the mapper never crashes or produces nonsensical output. |
| `10_missing_outliers` | present | Maps despite dropped/outlier measurements; precision stays perfect (`1.000`) since Phase 3 preprocessing already filters most bad measurements before they ever reach the mapper. |

## Ground Truth

From `scripts/evaluate_mapping.py` (5 scans each, seeded where the scenario has no seed of its
own):

| Scenario | Precision | Recall | IoU | False occupied | False free |
|---|---|---|---|---|---|
| `02_wall_in_front` | `1.000` | `0.728` | `0.728` | `0` | `78` |
| `03_pole_left` | `1.000` | `0.375` | `0.375` | `0` | `5` |
| `04_vehicle_ahead` | `1.000` | `0.429` | `0.429` | `0` | `24` |
| `05_multiple_obstacles` | `0.752` | `0.467` | `0.404` | `30` | `104` |
| `06_narrow_corridor` | `1.000` | `0.442` | `0.442` | `0` | `235` |
| `09_noisy_lidar` | `0.797` | `0.721` | `0.610` | `48` | `73` |
| `10_missing_outliers` | `1.000` | `0.727` | `0.727` | `0` | `76` |

Precision is `1.000` on every clean, single/simple-obstacle scenario -- the mapper essentially
never falsely claims OCCUPIED where there is nothing. `05_multiple_obstacles`'s lower precision
(`0.752`) reflects occlusion-fragment artifacts (one obstacle partially blocking another,
analogous to the same phenomenon docs/clustering.md documents for clustering) rather than a
mapping defect. Recall's cap well below `1.0` across every scenario is expected and explained
above ("Ground-truth evaluation" methodology) -- **not a claim of perfect accuracy.**

## Performance

`scripts/benchmark_mapping.py`, 200 scans per scenario:

| Scenario | Map-only avg | Map-only max | Full-chain avg | Full-chain scans/sec |
|---|---|---|---|---|
| `01_empty` | `15.2ms` | `20.2ms` | `17.8ms` | `56` |
| `02_wall_in_front` | `13.3ms` | `17.1ms` | `15.8ms` | `63` |
| `05_multiple_obstacles` | `14.3ms` | `21.8ms` | `17.0ms` | `59` |
| `07_moving_crossing` | `15.9ms` | `20.7ms` | `18.6ms` | `54` |
| `09_noisy_lidar` | `13.6ms` | `19.0ms` | `16.4ms` | `61` |

Peak memory (`05_multiple_obstacles`, 50 scans, `tracemalloc`): **2.16 MB**. Full-chain throughput
(54-63 scans/sec) is comfortably above the ~10Hz real-time target, with several times the needed
margin. The mapping stage itself (13-16ms avg) is the slowest single stage measured so far in
this project (vs. tracking's sub-millisecond -- see docs/tracking.md "Performance") because it
does genuine per-cell log-odds writes in a Python loop over every ray's traversed cells (up to a
few hundred cells per ray, ~360 rays/scan); this was measured, found comfortably within the
real-time budget as-is, and intentionally not further optimized per this phase's own instruction
("do not over-optimize prematurely") -- vectorizing the per-ray update loop (e.g. via numpy
fancy-indexing across all of one scan's touched cells at once) is a documented future option if
higher throughput is ever needed.

## Configuration

All in `common.config.Settings` (`perception/src/common/config.py`), `LIDAR_`-prefixed env-var
overridable, nothing hard-coded:

| Setting | Default | Meaning |
|---|---|---|
| `mapping_width_m` / `mapping_height_m` | `40.0` / `40.0` | Grid physical extent. |
| `mapping_resolution_m` | `0.1` | Cell size, meters. |
| `mapping_max_range_m` | `12.0` | Ray length trusted for free-space marking. |
| `mapping_no_return_margin_m` | `0.2` | How close to max range counts as "no return." |
| `mapping_free_update` / `mapping_occupied_update` | `0.4` / `0.85` | Log-odds update magnitudes. |
| `mapping_min_log_odds` / `mapping_max_log_odds` | `-2.0` / `3.5` | Clamp bounds. |
| `mapping_free_threshold` / `mapping_occupied_threshold` | `-0.3` / `0.7` | Discretization thresholds. |
| `mapping_decay_enabled` | `False` | Opt-in decay. |
| `mapping_decay_rate` | `0.02` | Per-update pull-toward-zero fraction. |

Full reasoning behind every default is in `common/config.py`'s own comments (matching this
project's established documentation-in-config pattern) and repeated in context throughout this
document.

## Example usage

```python
from coordinates import CoordinateTransformer
from mapping import OccupancyGridMapper
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

preprocessor, transformer = Preprocessor(), CoordinateTransformer()
mapper = OccupancyGridMapper()  # MUST be constructed once and reused across scans
source = make_data_source("02_wall_in_front")

with source:
    for _ in range(10):
        raw_scan = source.read_scan()
        clean_scan = preprocessor.process(raw_scan)
        cartesian_scan = transformer.transform(clean_scan)
        grid = mapper.update(cartesian_scan)

stats = mapper.get_statistics()
print(f"{stats.occupied_percentage:.1f}% occupied, {stats.free_percentage:.1f}% free, {stats.unknown_percentage:.1f}% unknown")
```

## Limitations

- **2D only.** See "Status" above -- this is a 2D LiDAR occupancy map, not a true 3D map. No
  height/elevation information is captured or inferred.
- **Fixed identity vehicle pose in this phase.** The `VehiclePose` abstraction is real and applied
  (see "Vehicle position"), but nothing in Phase 8 itself ever supplies a non-identity one --
  localization/SLAM is future scope.
- **Bresenham, not sub-cell-precise ray traversal.** See "Ray traversal" -- immaterial at this
  project's default resolution, a documented tradeoff.
- **Ground-truth recall is capped by single-viewpoint occlusion, not a mapping defect.** See
  "Ground-truth evaluation" -- this evaluation methodology cannot and does not claim perfect
  mapping accuracy.
- **Map boundary measurements are ignored, not clipped.** A measurement whose endpoint falls
  outside the configured grid contributes nothing to the map (see "Map boundaries") -- with the
  default grid comfortably exceeding sensor range, this essentially never triggers in normal use,
  but a badly undersized grid would silently lose those measurements' free-space evidence too
  (not just the endpoint), rather than the ray being clipped to the map edge.
- **Decay is a secondary mechanism in this phase's fixed-pose configuration.** See "Decay" --
  its practical effect today is limited (direct contradicting evidence already handles most
  moving-obstacle cases); it becomes considerably more relevant once vehicle pose genuinely
  changes scan-to-scan in a future phase.
- **Mapping stage performance (13-16ms avg) is not vectorized per-ray**, unlike most of this
  project's other numpy-heavy stages -- comfortably within the real-time budget as measured, but
  the highest-cost single stage so far; see "Performance" for the documented future optimization
  path if ever needed.
