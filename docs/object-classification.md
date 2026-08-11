# Geometric Object/Obstacle Classification

## Status

**Phase 6 — implemented.** Package: `perception/src/objects/`. Turns a
`models.clustering.ClusteredScan` into a `models.classification.ClassifiedScan`: each retained
cluster mapped onto a `models.objects.DetectedObject` with a geometry-based category, a
confidence score, the full extracted feature set, and a human-readable explanation. Has no
dependency on `simulator` or on any later pipeline stage (tracking, mapping, collision,
clearance) or on Unity/cloud -- same architecture boundary as every earlier phase, see
docs/architecture.md.

**This is geometry-based shape classification from a single 2D 360° LiDAR scan. It is not
general-purpose computer vision or object recognition, and it does not claim reliable human
identification.** Every category name reflects that: `VEHICLE_LIKE`, not `CAR`; `POLE_LIKE`, not
`POLE`; `PERSON_LIKE` is implemented but deliberately hobbled (see below). `UNKNOWN` is a
first-class, intentionally common outcome, not a failure mode.

## Why classification is required

Phase 5 groups points into `ObstacleCluster`s but says nothing about what each one *is* --
downstream consumers (a collision engine, a dashboard, a human operator) need at least a rough
category and a sense of how big/close/confident that estimate is. This phase adds exactly that,
using only the shape evidence a 2D LiDAR scan actually provides -- no camera, no learned model,
no assumption the object was ever seen before.

## Architecture

```text
ClusteredScan
      |
for each ObstacleCluster:
    extract_features()      -> ShapeFeatures        (objects.features)
    score_wall / score_pole /
    score_vehicle / score_person /
    score_large_obstacle()  -> five (score, reasons) pairs   (objects.scoring)
    pick the best (with a documented tie-break priority, see below)
    best >= classification_min_confidence ?
        yes -> that category, confidence = best score
        no  -> UNKNOWN, confidence = best score anyway (transparency, see "Confidence score")
    map ObstacleCluster + ShapeFeatures + classification -> DetectedObject
      |
ClassifiedScan
```

`GeometricClassifier.classify(clustered_scan)` is the public API (`objects.GeometricClassifier`);
`classify_scan(clustered_scan)` is a one-off convenience wrapper.

## Feature definitions

Computed once per cluster by `objects.features.extract_features` (`models.objects.ShapeFeatures`):

| Feature | Meaning |
|---|---|
| `point_count` | Member points. |
| `width`, `depth` | Bounding-box extent along X, Y (same values/convention as `ObstacleCluster.width/.depth`). |
| `aspect_ratio` | `max(width, depth) / max(min(width, depth), epsilon)`, always `>= 1`. `1` = square/compact, large = elongated. |
| `min_distance`, `max_distance`, `centroid_distance` | Passthrough from the cluster. |
| `min_angle`, `max_angle`, `angular_width` | Passthrough from the cluster (already 0/360-safe, Phase 5). |
| `mean_distance` | Mean of member points' polar distance. |
| `distance_variance` | Population variance of polar distance -- radial spread. |
| `spatial_variance` | Mean squared `(x, y)` distance from the centroid, m² -- overall 2D spread. |
| `point_density` | `point_count / max(width, depth, epsilon)` -- points per meter of major-axis extent. Chosen over an area-based density because a wall's bounding-box *area* can be near-zero (it's nearly 1D), which would blow area-based density up to an unstable, uninformative number; a *linear* density stays meaningful for both thin walls and compact poles. |
| `linearity_score` | `[0, 1]`, from a PCA/total-least-squares line fit -- see "Shape fitting". |
| `circularity_score` | `[0, 1]`, from an algebraic (Kasa) circle fit -- see "Shape fitting". |

### Investigated but not implemented

Per this phase's explicit instruction not to calculate unnecessarily expensive features:

- **Convex hull area / perimeter / isoperimetric circularity**: a LiDAR cluster is a set of
  *boundary/surface* points along the visible-from-here arc of an object, not a filled 2D
  footprint -- "the area enclosed by the hull of a 1D arc" is not a physically meaningful
  quantity the way it would be for, say, a segmented camera blob. Investigated and set aside;
  `linearity_score`/`circularity_score` from direct curve fitting are more directly meaningful
  for this data.
- **A dedicated oriented rectangle-fit residual**: a single 2D LiDAR viewpoint typically sees at
  most two faces of a box-shaped object (a corner), never all four -- fitting "the best rectangle"
  to that is underdetermined/fragile in a way a plain size + aspect-ratio + non-circular check
  (already used by `score_vehicle`) captures adequately without the added complexity and failure
  modes of a proper oriented-bounding-box search (e.g. rotating calipers).

## Shape fitting

`objects.shape_fitting`, both closed-form (non-iterative):

- **`line_fit_linearity`**: PCA on the point cloud's covariance matrix. The major eigenvalue is
  the spread *along* the best-fit line, the minor eigenvalue is the spread *perpendicular* to it
  (the fit residual). `linearity_score = 1 - minor/major`.
- **`circle_fit_circularity`**: Kasa's algebraic method solves a linear least-squares system for
  the circle's equation directly, giving center and radius with no iteration. Guarded against the
  degenerate case a flat/near-flat set of points produces (mathematically, a line is a circle of
  infinite radius): an **absolute** radius cap (default `2.0m`) rejects any fit whose radius is
  larger than any real "pole-like" obstacle plausibly has, regardless of how small its residual
  looks *relative to* that oversized radius.

**Why closed-form fits, not RANSAC** (which this phase's spec explicitly asks to investigate):
RANSAC earns its keep against significant outlier *contamination* a robust fit must ignore. By
the time points reach this stage, Phase 3 (preprocessing) has already removed sensor outliers and
Phase 5 (DBSCAN) has already separated noise from dense clusters -- a cluster's member points are
already a curated, low-contamination set. A direct closed-form fit gives the same practical
result without RANSAC's randomness or iteration cost, and -- being fully deterministic -- is
naturally easier to test and explain (a core requirement of this phase).

## Classification method

Five independent, explainable rule-based scores, each `[0, 1]`-ish, each returning its score
*and* a list of plain-language reasons (`objects.scoring`):

- **`score_wall`**: `linearity_score × extent_factor × flatness_factor`. `extent_factor` ramps
  from `0` at `classification_wall_min_length_m` to `1` at 2x that. `flatness_factor` requires the
  cluster's *other* (perpendicular) extent to be small (`classification_wall_max_thickness_m`) --
  added after testing against real scenario data showed a genuinely boxy/2-faced cluster (a
  rotated vehicle) could otherwise still pass a pure linearity+length check.
- **`score_pole`**: `circularity_score × compact_factor`, where `compact_factor` requires overall
  extent under `classification_pole_max_extent_m`.
- **`score_vehicle`**: a weighted combination -- but **gated multiplicatively** by a size check
  (`size_factor`), not merely one additive term among several. Checked against *both* the
  cluster's primary (larger) and secondary (smaller) extent, taking whichever fits the configured
  vehicle-width range better: a flat-face view (common -- a 2D LiDAR usually sees only a vehicle's
  near side) puts the meaningful dimension in the primary extent; a cornered/boxy view can inflate
  the primary extent via axis-aligned-bounding-box-of-a-rotated-shape effects (see
  docs/clustering.md "Known limitations"), in which case the secondary extent is the more
  reliable one. The remaining "quality" terms (secondary-dimension plausibility, point density,
  non-circularity) only refine the score *within* that size gate -- see "Parameter selection" for
  why the gate had to be multiplicative, not additive.
- **`score_person`**: see "Person-like classification" below -- deliberately the most hedged.
- **`score_large_obstacle`**: `extent_factor × (1 - best_other_score)` -- a genuine catch-all,
  only activating where the four more specific categories don't already explain the cluster well.

The winner is the highest score; ties within `0.02` are broken by a fixed, documented priority
(`WALL > POLE_LIKE > VEHICLE_LIKE > LARGE_OBSTACLE > PERSON_LIKE`) rather than incidental
floating-point ordering, which was observed to flip unpredictably between near-identical scores
during testing. The reasoning: `WALL`/`POLE_LIKE` require only shape evidence (no assumption
about a specific real-world size range actually matching this obstacle); `VEHICLE_LIKE` and the
others additionally assume a configured size envelope applies, a more specific and so more
fragile claim, and lose ties to the more general one.

If the winning score is `>= classification_min_confidence`, that's the classification. Otherwise:
**`classification = UNKNOWN`**, exactly as this phase's spec requires -- never a forced,
under-supported label.

## Confidence score

`DetectedObject.confidence` is the winning rule's raw `[0, 1]` score -- **not** a calibrated
statistical probability (this phase's spec is explicit about this distinction: it is a
`confidence_score` reflecting evidence quality via an explainable formula, not "an 82% chance
this is a car" in any rigorous probabilistic sense). Even for `UNKNOWN`, `confidence` is set to
the best candidate's score (not `0.0` and not hidden) so a consumer can see *how close* it came
and why it still didn't qualify -- visible directly in `classification_reason`.

## Explainability

Every `DetectedObject.classification_reason` is a list of plain-language strings: the winning
label and score on the first line, then the specific evidence (e.g. `"linearity 0.98
(total-least-squares line fit)"`, `"extent 6.20m (>= 1.00m minimum wall length)"`). For `UNKNOWN`,
the first line names the best candidate, its score, and the threshold it fell short of. See
`scripts/visualize_classification.py --explain` for a formatted printout.

## Person-like classification

Treated exactly as cautiously as this phase's spec requires:

- Size gated to a narrow, configurable range (`classification_person_extent_min_m` /
  `_max_m`, default `[0.15, 0.9]m`).
- Requires *both* `circularity_score` and `linearity_score` to be low (the *minimum* of their two
  complements, not merely their difference -- see "Parameter selection" for why the naive
  "similar to each other" version was a false-positive trap for small clusters).
- **Hard-capped** at `classification_person_max_confidence` (default `0.6`) regardless of the raw
  computed score -- a `PERSON_LIKE` result can never claim high confidence.
- **The simulator has no person-shaped obstacle scenario.** `PERSON_LIKE` is validated only by
  synthetic unit tests in this phase, never observed or tuned against a realistic scan. This is
  stated plainly, not glossed over: a single 2D LiDAR scan's shape signature for "a person" and
  "some other small irregular object" (a fence post, a trash bin, a bollard) can be
  indistinguishable, and this classifier makes no claim otherwise.

## Configuration

All on `common.config.Settings` (see `.env.example`), overridable via `LIDAR_`-prefixed
environment variables or per-`GeometricClassifier`-instance:

| Setting | Default | Meaning |
|---|---|---|
| `classification_min_confidence` | `0.55` | The one threshold deciding "confident category" vs. `UNKNOWN`. |
| `classification_wall_min_length_m` | `1.0` | Below this, never confidently `WALL` even if perfectly linear. |
| `classification_wall_max_thickness_m` | `0.6` | Above this perpendicular extent, not flat enough to be `WALL`. |
| `classification_pole_max_extent_m` | `0.8` | Above this, too big to be `POLE_LIKE` even if circular. |
| `classification_vehicle_width_min_m` / `_max_m` | `1.0` / `2.6` | The visible-face size range `VEHICLE_LIKE` checks against. |
| `classification_vehicle_depth_max_m` | `6.0` | Upper bound the *secondary* (non-primary) extent must stay under. |
| `classification_vehicle_min_points` | `6` | Point-count floor contributing to `VEHICLE_LIKE`'s quality term. |
| `classification_person_extent_min_m` / `_max_m` | `0.15` / `0.9` | The narrow `PERSON_LIKE` size range. |
| `classification_person_max_confidence` | `0.6` | Hard cap on `PERSON_LIKE` confidence, regardless of score. |
| `classification_large_min_extent_m` | `1.2` | Minimum extent for `LARGE_OBSTACLE`'s catch-all. |

## Parameter selection (the actual process)

Every threshold above was reached the same way Phase 5's DBSCAN parameters were: a theoretical
starting point, then empirical testing against every required scenario (not just one), with real
bugs found and fixed along the way. Highlights:

1. **`linearity_score` alone is not very discriminative.** Measured directly: a real wall, a real
   pole (5 points), and a real vehicle's flat face all had `linearity_score >= 0.95` -- because a
   small/short cluster's PCA fit trivially finds *some* dominant axis regardless of true shape.
   `circularity_score` (with the absolute radius cap) turned out far more discriminative in
   practice.
2. **The circularity radius guard needed to be absolute, not relative to the cluster's own
   extent.** An earlier version capped the fitted radius at `5x` the cluster's own extent; tested
   against real data, a wall's fit radius came out ~60m (correctly rejected either way), but a
   *short* flat vehicle face's fit radius (~7m) was still under `5x` its own short (~1.7m) extent
   -- passing the guard and scoring as falsely circular. Measured real fit radii (wall ~60m,
   vehicle face ~7m, real poles ~0.15-0.4m) showed an absolute cap (`2.0m`) separates these
   cleanly where a per-cluster ratio did not.
3. **A pure size-range check let occluded wall fragments score as confidently `VEHICLE_LIKE` as a
   real vehicle.** `05_multiple_obstacles`'s background wall is fragmented into two pieces by an
   occluding pole (see docs/clustering.md); one fragment's length (~2.4m) coincidentally falls
   inside a plausible vehicle-width range. Investigated two complementary fixes, both applied:
   (a) `score_wall` gained a flatness gate (a genuinely boxy/rotated vehicle cluster has
   substantial *perpendicular* extent too, unlike a flat wall fragment); (b) `score_vehicle`'s
   size check was changed from a wide *union* of the width and depth ranges (any 1-6m extent) to
   the narrower width range alone, checked against *both* the cluster's primary and secondary
   extent. Some genuine ambiguity remains for fragment lengths that coincidentally match a
   vehicle's width almost exactly -- see "Known failure cases" below; this is a real, physically
   grounded ambiguity, not something further threshold tuning can fully resolve.
4. **`size_factor` had to be a multiplicative gate, not an additive term.** Measured directly: a
   long noisy wall (`extent_factor` correctly `0` for `VEHICLE_LIKE`) still scored `~0.60` --
   over the confidence threshold -- purely from the other three weighted terms (point count,
   secondary-extent leniency, non-circularity), none of which actually required *any* size
   evidence. Multiplying by `size_factor` instead of adding it fixed this: zero size evidence now
   means zero score, full stop.
5. **The naive `PERSON_LIKE` "irregularity" formula (`1 - |circularity - linearity|`) was a false
   positive trap.** For small (~5-point) clusters, both scores are often *simultaneously* high
   (PCA/Kasa both trivially "explain" a handful of points well, regardless of true shape) --
   rewarding their *similarity* gave a real pole a `PERSON_LIKE` score competitive with its own
   `POLE_LIKE` score. Requiring the *minimum* of the two complements (both must be low, not merely
   close to each other) fixed this directly.
6. **Tie-breaking needed to be explicit.** Two near-identical scores (both effectively `1.0`
   after clipping) were observed to select different winners run to run purely from floating-point
   rounding in the two different formulas. A documented priority order (see "Classification
   method") replaced reliance on incidental float ordering.

## Test results

278 pre-existing tests (Phases 0-5) still pass unchanged. New: **61 tests** in `perception/tests/`
(`test_objects_shape_fitting.py`, `test_objects_features.py`, `test_objects_classifier.py` -- the
spec's synthetic Wall/Pole/Vehicle/Large-obstacle/Ambiguous cases plus model-field mapping,
confidence bounds, explainability, and determinism, `test_objects_metrics.py`) and **19
integration tests** in `simulator/tests/test_classification_integration.py` covering all 9
required scenarios. **358 tests total, all passing.**

## Scenario results

One representative run per scenario (see docs/testing.md for exact reproducibility notes --
scenarios without a fixed noise seed vary slightly run to run):

| Scenario | Objects | Classification(s) | Confidence |
|---|---|---|---|
| `02_wall_in_front` | 1 | `WALL` | 0.88 |
| `03_pole_left` | 1 | `POLE_LIKE` | 0.60-0.72 |
| `04_vehicle_ahead` | 1 | `VEHICLE_LIKE` | 0.78-1.00 |
| `05_multiple_obstacles` | 5 | `VEHICLE_LIKE`, `POLE_LIKE`, `WALL`, `POLE_LIKE`, `VEHICLE_LIKE` | 0.56-1.00 |
| `06_narrow_corridor` | 2 | `WALL`, `WALL` | 0.90-0.93 |
| `07_moving_crossing` | 1 (3-4 pts) | `UNKNOWN` (occasionally `POLE_LIKE`) | 0.00-0.6ish |
| `08_approaching_obstacle` | 1 | `VEHICLE_LIKE` (stable across 5 scans) | 1.00 |
| `09_noisy_lidar` | 1 | `WALL` or `LARGE_OBSTACLE` (noise-dependent) | 0.5-0.9 |
| `10_missing_outliers` | 2-4 | mostly `WALL`, small fragments sometimes `VEHICLE_LIKE` | 0.56-0.96 |

Full per-object detail (features, reasons) via
`python scripts/visualize_classification.py --scenario <id> --explain`.

## Classification quality metrics

`objects.metrics.classification_metrics` (accuracy, per-class + macro precision/recall/F1,
confusion matrix) is generic and simulator-independent, unit-tested with synthetic label lists
(`perception/tests/test_objects_metrics.py`). `scripts/evaluate_classification.py` supplies the
scenario-specific *ground truth* (known from how each scenario was built) and runs the full
pipeline against it -- one representative run:

```text
Accuracy: 0.647   Macro precision: 0.500   Macro recall: 0.420   Macro F1: 0.417
wall:         precision=1.000 recall=0.600 f1=0.750 support=10
pole_like:    precision=1.000 recall=0.500 f1=0.667 support=4
vehicle_like: precision=0.500 recall=1.000 f1=0.667 support=3
```

**Read this carefully, per this phase's explicit instruction:** these numbers are computed over
only **17 objects across 9 scenarios in one run**, and only for the three classes that scenario
lets us construct ground truth for (`wall`, `pole_like`, `vehicle_like`) -- `PERSON_LIKE` and
`LARGE_OBSTACLE` have **no evaluated ground truth at all** (no scenario contains one by design),
and `UNKNOWN` is never a *true* label here (it's a possible *prediction*, correctly counted as
wrong when the true label was something specific). Several scenarios have no fixed noise seed, so
re-running `scripts/evaluate_classification.py` will shift these exact numbers -- the *pattern*
(near-perfect on `02`/`03`/`04`/`06`/`08`, imperfect on the occlusion-fragmented `05` and the
noise-elevated `09`/`10`) is the stable, meaningful result, not the third decimal place.

## Performance

Measured with `scripts/benchmark_classification.py` (feature extraction + classification time
only; preprocessing/coordinates/clustering time excluded, see their own benchmark scripts), 300
scans/scenario:

| Scenario | features avg (ms) | classify avg (ms) | total avg (ms) | total max (ms) |
|---|---|---|---|---|
| `01_empty` | 0.0005 | 0.011 | 0.013 | 0.031 |
| `02_wall_in_front` | 0.297 | 0.289 | 0.587 | 1.357 |
| `05_multiple_obstacles` | 0.475 | 0.573 | 1.049 | 3.681 |
| `09_noisy_lidar` | 0.314 | 0.338 | 0.652 | 1.598 |
| `10_missing_outliers` | 0.301 | 0.325 | 0.627 | 1.376 |

Overall average ~0.59ms/scan (~1700 scans/second), comfortably over the 10+ scans/second
real-time target.

## Visualization

`objects.visualize.plot_classified_scan` (optional `matplotlib`, `pip install -e
"./perception[viz]"`): each object color-coded by category, with its bounding box, centroid, and
a `LABEL confidence` label; noise points shown separately.

```bash
python scripts/visualize_classification.py --scenario 05_multiple_obstacles --visualize
python scripts/visualize_classification.py --scenario 04_vehicle_ahead --visualize --explain
python scripts/evaluate_classification.py
```

## Example usage

```python
from clustering import DBSCANClusterer
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

preprocessor, transformer, clusterer, classifier = Preprocessor(), CoordinateTransformer(), DBSCANClusterer(), GeometricClassifier()
source = make_data_source("04_vehicle_ahead")

with source:
    raw_scan = source.read_scan()
    clean_scan = preprocessor.process(raw_scan)
    cartesian_scan = transformer.transform(clean_scan)
    clustered_scan = clusterer.cluster(cartesian_scan)
    classified_scan = classifier.classify(clustered_scan)

for obj in classified_scan.objects:
    print(obj.classification.value, obj.confidence, obj.classification_reason)
```

## Known failure cases

- **Occluded wall fragments can be mistaken for vehicles.** When one obstacle occludes part of
  another (see docs/clustering.md), a resulting wall *fragment*'s length can coincidentally match
  the configured vehicle-width range closely enough to outscore `WALL`. This is a genuine,
  physically-grounded ambiguity -- a short, flat, linear segment's *shape* is identical whether
  it's a wall piece or a vehicle's side profile; 2D geometry alone cannot always distinguish them.
  Resolving this fully would need information this phase doesn't have (temporal consistency
  across scans -- Phase 7's job -- or sensor fusion).
- **Elevated sensor noise can push a real wall's measured "thickness" over the flatness gate**,
  causing it to fall back to `LARGE_OBSTACLE` (or occasionally `UNKNOWN`) instead of a confident
  `WALL`. This is a *safe* degradation (a generic, honest label instead of a wrong specific one),
  not a crash or a false positive, but it does mean confidence in the *specific* category name
  drops under noise even when the object is still clearly "something."
- **Very small clusters (near the Phase 5 minimum of 3 points) produce unstable shape fits.**
  Both `linearity_score` and `circularity_score` can come out simultaneously high for a handful
  of points regardless of true shape (either fit trivially "explains" very few points well) --
  `score_person`'s formula was specifically redesigned around this (see "Parameter selection"
  point 5), but the underlying instability affects all scores computed from tiny clusters, most
  visibly in `07_moving_crossing`'s 3-4-point moving pole, which is often (correctly, safely)
  `UNKNOWN`.
- **`PERSON_LIKE` has zero real-scenario validation.** See "Person-like classification" above --
  stated here again because it's the single most important limitation of this phase.
- **Axis-aligned bounding-box inflation for rotated objects** (documented in docs/clustering.md)
  propagates into classification: a rotated vehicle's true ~1.8m×4.5m footprint can present as a
  larger axis-aligned box, which `score_vehicle`'s dual (primary/secondary) size check mitigates
  but does not eliminate.
- **No shape memory across scans.** Every classification is computed from that single scan's
  cluster alone; a real object's classification can (rarely) flicker between adjacent scans if
  its measured geometry shifts near a threshold. Temporal smoothing/voting is a natural Phase 7
  extension, not implemented here.
