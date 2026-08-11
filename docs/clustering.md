# Obstacle Clustering and Segmentation

## Status

**Phase 5 — implemented.** Package: `perception/src/clustering/`. Turns a
`models.coordinates.CartesianScan` into a `models.clustering.ClusteredScan`: groups of points
believed to belong to the same physical obstacle. Has no dependency on `simulator` or on any
later pipeline stage (object classification, tracking, mapping, collision, clearance) or on
Unity/cloud -- same architecture boundary as `preprocessing` and `coordinates`, see
docs/architecture.md.

**This phase does not classify obstacles.** A cluster carries only geometry (points, centroid,
bounding box, distances, angular extent) -- whether it's a wall, pole, vehicle, or person is
Phase 6.

## Why clustering is required

A `CartesianScan` is just a flat list of individual `(x, y)` points -- there is no notion yet of
"these 40 points are one wall" versus "these 5 points are a pole." Every later stage (shape
classification, tracking, collision/clearance analysis) needs to reason about *obstacles*, not
*individual measurements*: an obstacle's shape, how many there are, how they move over time, and
how close the nearest one is are all cluster-level questions. Clustering is the step that turns
"a cloud of points" into "a set of candidate obstacles," without yet saying anything about what
each one is.

## Algorithm selected: DBSCAN

Investigated per this phase's instructions: Euclidean distance-based clustering, adjacent-point
scan-based segmentation, and DBSCAN. **DBSCAN was selected** (as directed, absent a clear reason
to prefer another approach -- and testing confirmed there wasn't one). Why it fits this problem
specifically:

| Requirement | Why DBSCAN fits |
|---|---|
| Unknown number of objects | DBSCAN discovers however many dense regions exist; it isn't told a target cluster count up front (unlike k-means). |
| Irregular obstacle shapes | DBSCAN makes no shape assumption -- it groups by spatial density, not by fitting a shape template (a wall, a pole, and a rotated rectangle all just need "many points close together," nothing more). |
| Noise | DBSCAN has a built-in notion of noise -- points that aren't part of any dense region are labeled as such, rather than being forced into the nearest cluster regardless of fit. |
| Variable cluster sizes | A thin pole (~5 points) and a long wall (100+ points) are both valid DBSCAN clusters; it has no fixed-size or similar-size assumption. |
| 2D LiDAR specifically | Operating on `(x, y)` (not `(angle, distance)`) means the clustering *decision* has no 0/360 discontinuity at all -- see "0/360 handling" below. |

Adjacent-point/scan-based segmentation was not chosen as the primary method because it only
considers a point's immediate angular neighbors; it cannot "reach across" an angular gap left by
an occluding obstacle (see "Known limitations" below) or naturally express density in 2D space.
Plain Euclidean single-linkage clustering (no density/noise concept) was not chosen because it
has no built-in way to reject sparse/spurious points as noise -- every point would need to join
*some* cluster. No machine learning was used, per this phase's instructions.

## DBSCAN, briefly

For each point, DBSCAN checks how many other points lie within a radius `eps`. A point with at
least `min_samples` points (including itself) in that radius is a **core point**. Core points
within `eps` of each other are chained into the same cluster, and any point within `eps` of a
core point (even if not a core point itself) joins that cluster too. Points that end up in no
core point's neighborhood are **noise**. Implemented via `sklearn.cluster.DBSCAN` (a
well-tested, efficient implementation) -- not hand-rolled.

## Parameters

All on `common.config.Settings` (see `.env.example`), overridable via `LIDAR_`-prefixed
environment variables or per-`DBSCANClusterer`-instance:

| Setting | Default | Meaning |
|---|---|---|
| `clustering_eps_m` | `0.6` | DBSCAN neighborhood radius, meters. |
| `clustering_min_samples` | `3` | Points (including itself) required within `eps` for a point to be a core point. |
| `clustering_min_cluster_points` | `3` | Independent post-filter: a DBSCAN cluster smaller than this is demoted to noise. Equals `min_samples` by default, making it a no-op (every DBSCAN cluster already has `>= min_samples` points by construction) -- raise it above `min_samples` for extra noise robustness without changing DBSCAN's own density parameter. |
| `clustering_max_range_margin_m` | `0.2` | Points within this margin of `lidar_range_max_m` are treated as free-space/no-return and excluded from clustering entirely -- see "Free-space filtering" below. |

## Parameter selection (the actual process, not just the final numbers)

**Naive starting point.** The obvious first estimate: `eps` should comfortably bridge the gap
between angularly-adjacent points on one surface. At the simulator's default 1° angular
resolution, the worst-case adjacent-point arc spacing at the default 12m max range is
`12 * radians(1°) ≈ 0.21m`. `eps_m=0.3` (with a conventional `min_samples=4`) looked like a safe
margin above that.

**This was wrong in practice**, discovered by actually running clustering against every
simulator scenario (not just one) before finalizing anything, per this phase's instruction not
to "blindly optimize parameters for only one scenario":

```text
eps=0.3, min_samples=4:
02_wall_in_front         clusters= 1  (85-88 points -- looked OK at first glance)
09_noisy_lidar           clusters= 5  (67, 11, 4, 5, 15 -- a single wall fragmented into 5 pieces)
```

Investigating *why* revealed the actual mechanism: LiDAR points on one surface form a **quasi-1D
arc**, not a 2D blob. A point's `eps`-radius neighborhood on that arc mostly only contains its
immediate line-neighbors -- at `eps=0.3` and the ~0.16-0.21m adjacent spacing typical in these
scenarios, a point typically has only its immediate ±1 neighbor within range (2 neighbors + 
itself = 3 points), one short of `min_samples=4`'s requirement of *3 other* points. The naive
calculation only checked "does `eps` bridge one gap to the next point" -- it didn't check "does
`eps` reach enough neighbors to satisfy `min_samples`," which is a different, stricter condition
for line-shaped point distributions. With no core points, an *entire real wall* was classified as
100% noise in some runs.

**Resolution, verified two independent ways** (both work; the shipped defaults use the first):
raising `eps` so a point's neighborhood reaches enough neighbors (`eps_m=0.6`, roughly `2x` the
worst-case spacing, reaching `±2` neighbors), or lowering `min_samples` to match what `eps=0.3`
can actually reach (`min_samples=3`). See
`perception/tests/test_clustering_parameters.py::TestQuasi1DDensityInsight` for both, verified
against a synthetic quasi-1D arc at the exact radius that first exposed this.

**Final validation against every scenario** with `eps_m=0.6, min_samples=3`:

```text
01_empty                 clusters= 0   noise=360
02_wall_in_front         clusters= 1   noise=233   sizes=[127]
03_pole_left             clusters= 1   noise=355   sizes=[5]
04_vehicle_ahead         clusters= 1   noise=333   sizes=[27]
05_multiple_obstacles    clusters= 5   noise=266   sizes=[44, 24, 16, 5, 5]
06_narrow_corridor       clusters= 2   noise= 72   sizes=[144, 144]
07_moving_crossing       clusters= 1   noise=357   sizes=[3]     (per scan, 5 scans checked)
08_approaching_obstacle  clusters= 1   noise=347   sizes=[13]    (per scan, 5 scans checked)
09_noisy_lidar           clusters= 1   noise=235   sizes=[125]
10_missing_outliers      clusters= 4   noise=197   sizes=[85, 14, 4, 3]
```

**Over-merge boundary, checked explicitly** (the "too-large eps" pitfall): the narrow corridor's
two walls are 2.0m apart. Sweeping `eps` from the default up: clusters stay separate (2) all the
way to `eps=1.9m`, and only merge into 1 at `eps=2.0m` -- the default of `0.6m` has more than 3x
headroom before that failure mode.

**Too-small/too-large sweeps, checked explicitly** (spec section 11), all reproduced as exact
tests in `perception/tests/test_clustering_parameters.py`:

| Case | Effect (measured) |
|---|---|
| `eps` too small (`0.02`-`0.1m`) | A whole wall becomes 100% noise, or fragments into many small pieces. |
| `eps` too large (`>=` the real gap between two obstacles) | Two genuinely distinct obstacles merge into one cluster. |
| `min_samples` too small (`1`) | Every isolated point can become its own trivial 1-point "cluster" -- `min_cluster_points` (default `3`) is what actually screens these out, since a single point carries no shape information. |
| `min_samples` too large (`100`, or `15` even at `eps=0.6`) | No point can ever reach that many neighbors -- everything becomes noise, even a real, dense wall. |

**These are the simulator's defaults, not universal constants.** They're tuned to the default
`1°` angular resolution and `12m` max range; a different sensor resolution or range would shift
the adjacent-point spacing and should be re-validated the same way (run the full scenario suite,
don't tune against one case).

## Cluster representation

`models.clustering.ObstacleCluster` (see docs/data-model.md for the full field table). Reuses
`CartesianPoint` for member points (no parallel point representation). Every field the spec asks
for is present: `cluster_id`, `points`, `point_count`, `centroid_x`/`centroid_y`,
`min_x`/`max_x`/`min_y`/`max_y`, `width`/`depth`, `min_distance`/`max_distance`,
`centroid_distance`, `min_angle`/`max_angle`/`angular_width`, `timestamp`.

**Field-naming note (important, deliberately not silently resolved):** this phase's spec defines
`width = max_x - min_x` (the vehicle-forward/X extent) and `depth = max_y - min_y` (the
vehicle-lateral/Y extent). `models.objects.DetectedObject` (Phase 0) defined `width` as the
*lateral* extent and `depth` as the *radial* extent -- the opposite axis mapping. Both are
implemented exactly as their own phase specified them; this is a genuine inconsistency between
two phases' specs, not a bug in either. **Recommendation for Phase 6**, which will likely
populate a `DetectedObject` from an `ObstacleCluster`: do not copy `width`/`depth` across
unchanged -- explicitly decide (and document) which axis each target field should represent, and
swap if needed.

`ObstacleCluster.width`/`.depth` are always axis-aligned bounding-box extents, regardless of an
obstacle's true orientation (e.g. a rotated rectangle's bounding box is larger than the rectangle
itself -- visible in the `05_multiple_obstacles` visualization). No oriented/rotated bounding box
is computed in this phase.

`ClusteredScan` (top-level result): `scan_id`/`sequence_number`/`source_id`/`timestamp`
(passthrough), `clusters`, `noise_points`, `cluster_count`, `noise_count`, `total_points`,
`clustered_points`, `noise_percentage`, `average_cluster_size`, `largest_cluster_size`,
`smallest_cluster_size` -- covering both the spec's "cluster filtering" bookkeeping (section 7)
and "cluster quality metrics" (section 12) without a redundant nested wrapper duplicating
`cluster_count`/`noise_count` in two places.

## Noise handling

Three distinct things all end up in `ClusteredScan.noise_points` (nothing is silently deleted --
every input point is accounted for in exactly one of `clusters` or `noise_points`):

1. **DBSCAN-labeled noise** (`label == -1`): points not reachable from any core point.
2. **Sub-minimum clusters**: a DBSCAN cluster smaller than `clustering_min_cluster_points` is
   demoted to noise as a whole (see "Parameters" above).
3. **Free-space / "no return" points** (see next section): excluded from clustering entirely,
   counted as noise since they don't represent any obstacle.

## Free-space filtering

**Necessary, not optional.** A `CartesianScan` typically contains far more "no return" points
(the simulator reports `lidar_range_max_m` with `valid=True` for angles with nothing in range --
see docs/simulation.md) than obstacle-hit points. At the default resolution, adjacent free-space
points around that ring are only ~0.2m apart -- *closer together* than many real obstacle
points -- so naively feeding the whole scan to DBSCAN would happily connect the entire ring into
one large false "obstacle" cluster representing nothing. Verified directly: `01_empty` (no real
obstacles at all) must produce `0` clusters, and does, specifically *because* of this filter --
without it, DBSCAN would find one giant ring-shaped cluster instead.

Points with `distance >= lidar_range_max_m - clustering_max_range_margin_m` are excluded from
the clustering input and counted as noise (`clustering.dbscan.DBSCANClusterer.cluster`, "Free
space" section). This reuses the same `lidar_range_max_m` `Settings` field the simulator and
preprocessing already use -- no duplicate range concept introduced.

## 0°/360° handling

**The clustering *decision* itself needs no special handling.** DBSCAN operates on `(x, y)`,
and Cartesian space has no discontinuity at angle=0/360 -- a wall crossing that boundary is
simply a continuous line segment in `(x, y)`, exactly like any other wall. This is *why*
clustering on `(x, y)` (as this phase's instructions require) rather than on raw
`(angle, distance)` sidesteps the boundary problem that Phase 3's angle-ordered algorithms had to
solve explicitly with circular windowing.

The **angular-extent summary statistic** computed per cluster afterward (`min_angle`, `max_angle`,
`angular_width`) does need explicit circular handling, since raw angle values (359 vs. 1) are
numerically far apart despite being physically adjacent --
`clustering.geometry.circular_angular_extent` finds the shortest arc containing every member
angle by locating the single largest angular gap and treating everything else as the cluster's
span. Verified directly with the spec's exact example (`358°, 359°, 0°, 1°, 2°` → one cluster,
`min_angle=358`, `max_angle=2`, `angular_width=4`), both as a standalone geometry test
(`perception/tests/test_clustering_geometry.py`) and as a full DBSCAN clustering test with real
`(x, y)` coordinates at those angles (`test_clustering_dbscan.py::TestD_...`).

## Test results

231 pre-existing tests (Phases 0-4) still pass unchanged. New: **45 tests** in
`perception/tests/` (`test_clustering_geometry.py`, `test_clustering_dbscan.py` -- the spec's
Test A-E plus cluster-model/free-space/determinism checks, `test_clustering_parameters.py`) and
**11 integration tests** in `simulator/tests/test_clustering_integration.py` covering all 10
required scenarios. **287 tests total, all passing.** See docs/testing.md for the full breakdown
and "Scenario results" below for the numbers each integration test encodes.

## Scenario results

| Scenario | Clusters | Notes |
|---|---|---|
| `01_empty` | 0 | Free-space filtering does its job -- no obstacles, no clusters. |
| `02_wall_in_front` | 1 | The wall's full hit-cone (127 points), not fragmented. |
| `03_pole_left` | 1 | A thin pole -- only ~5 points, still correctly grouped, not treated as noise. |
| `04_vehicle_ahead` | 1 | The vehicle-like rectangle's visible face. |
| `05_multiple_obstacles` | 5 | Both poles and the rotated rectangle each their own compact cluster; the background wall (occluded by one pole from the LiDAR's viewpoint) becomes two separate wall-segment clusters -- a real perceptual limitation (occlusion), not a clustering defect, see "Known limitations." |
| `06_narrow_corridor` | 2 | The two 2m-apart walls are correctly kept separate. |
| `07_moving_crossing` | 1 per scan | The small moving pole forms one coherent cluster in every scan checked. |
| `08_approaching_obstacle` | 1 per scan | Stays one coherent cluster as centroid distance measurably shrinks scan to scan. |
| `09_noisy_lidar` | 1 | Elevated Gaussian noise (std=0.15m) does *not* fragment the wall at the tuned defaults. |
| `10_missing_outliers` | 4 | Elevated missing/outlier rates cause some fragmentation (a large 85-point main cluster plus 3 small fragments) -- bounded, not catastrophic. |

## Visualization

`clustering.visualize.plot_clustered_scan` (optional `matplotlib`, `pip install -e
"./perception[viz]"`): each cluster in a distinct color with its axis-aligned bounding box,
centroid marker, and an ID/point-count/distance label; noise points shown as gray "x" markers
separately. Debugging/algorithm-validation tool only -- not the future Unity digital twin.

```bash
python scripts/visualize_clusters.py --scenario 05_multiple_obstacles --visualize
python scripts/visualize_clusters.py --scenario 06_narrow_corridor --visualize
```

## Performance

Measured with `scripts/benchmark_clustering.py` (clustering time only, preprocessing and
coordinate transformation excluded -- see their own benchmark scripts), 300 scans/scenario,
360-point scenarios:

| Scenario | avg (ms) | max (ms) | min (ms) |
|---|---|---|---|
| `01_empty` | 0.04 | 0.18 | 0.03 |
| `02_wall_in_front` | 1.14 | 5.79 | 0.82 |
| `05_multiple_obstacles` | 1.24 | 3.64 | 0.84 |
| `09_noisy_lidar` | 1.21 | 3.38 | 0.84 |
| `10_missing_outliers` | 1.13 | 2.28 | 0.79 |

Overall average ~0.95ms/scan (~1000 scans/second), comfortably over the 10+ scans/second
real-time target. `01_empty` is fastest since the free-space filter excludes every point before
`sklearn.cluster.DBSCAN` is even called.

## Example usage

```python
from clustering import DBSCANClusterer
from coordinates import CoordinateTransformer
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

preprocessor = Preprocessor()
transformer = CoordinateTransformer()
clusterer = DBSCANClusterer()  # reads defaults from common.config.Settings
source = make_data_source("05_multiple_obstacles")

with source:
    raw_scan = source.read_scan()
    clean_scan = preprocessor.process(raw_scan)
    cartesian_scan = transformer.transform(clean_scan)
    clustered_scan = clusterer.cluster(cartesian_scan)

for cluster in clustered_scan.clusters:
    print(cluster.cluster_id, cluster.point_count, cluster.centroid_x, cluster.centroid_y)
```

One-off convenience: `from clustering import cluster_scan; cluster_scan(cartesian_scan)`.

## Known limitations

- **Occlusion fragments obstacles that are genuinely occluded from the LiDAR's single viewpoint**
  (`05_multiple_obstacles`: the background wall becomes two clusters because a pole stands in
  front of part of it, physically blocking those rays). This is a correct reflection of what a
  single 2D LiDAR scan can actually see, not a clustering algorithm defect -- a real sensor has
  the same limitation. Resolving "these are actually one wall" would require prior/semantic
  knowledge this phase deliberately does not have (Phase 5 is geometry-only).
- **No persistent identity across scans.** `cluster_id` is stable only *within* one scan (`0, 1,
  2, ...`); the same physical obstacle can and will get a different `cluster_id` next scan. That
  is Phase 7 (tracking), explicitly out of scope here.
- **Axis-aligned bounding boxes only.** A rotated obstacle's `width`/`depth` describe its
  axis-aligned bounding box, not its true oriented extent -- visibly larger than the object
  itself for a rotated rectangle (see the `05_multiple_obstacles` visualization).
- **`width`/`depth` axis convention differs from `DetectedObject`'s** -- see "Cluster
  representation" above; a documented spec inconsistency, not an oversight.
- **Tuned defaults are resolution/range-dependent.** `eps_m=0.6`/`min_samples=3` were validated
  against the simulator's default 1°/12m configuration; a different LiDAR configuration changes
  the adjacent-point spacing math and should be re-validated the same way (see "Parameter
  selection").
- **A single point cannot form a cluster** (`min_cluster_points >= 1` always demotes true
  1-point groups unless explicitly configured down to `1`, which is not the default) -- by
  design, since a lone point carries no shape/size information at all.
