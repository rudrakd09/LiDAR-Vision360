# LiDAR Data Preprocessing

## Status

**Phase 3 — implemented.** Package: `perception/src/preprocessing/`. Turns a raw
`models.scan.ScanFrame` into a clean `models.preprocessing.PreprocessedScan`. Has no dependency
on `simulator` or on any later pipeline stage (coordinates, clustering, tracking, mapping,
collision, clearance) or on Unity/cloud -- see "Architecture" below.

## Why preprocessing is required

Raw LiDAR measurements -- simulated today, real STM32/UART hardware later -- are not directly
usable by geometry-based algorithms (coordinate conversion, clustering, tracking):

- Sensors report out-of-range, non-finite, or explicitly-invalid ("no return"/dropout)
  measurements that must be identified and accounted for, not silently trusted.
- Individual measurements can be **outliers**: spurious single-point reflections wildly different
  from their surroundings, which would otherwise appear as phantom obstacles.
- Measurements carry sensor noise (small random jitter around the true distance), which would
  otherwise fragment a single flat surface into many spurious micro-clusters downstream.
- Nothing about a raw scan guarantees a stable angle ordering or a fixed point count -- later
  stages (especially clustering, which reasons about angular adjacency) need both.

Preprocessing is the one place all of this is handled, once, so every later stage can assume
clean, validated, angle-ordered input regardless of whether it came from the simulator or real
hardware.

## Architecture

```text
Raw ScanFrame
      |
Validation            (validation.classify_point / validate_points)
      |
Range Filtering        (folded into validation -- reuses lidar_range_min_m/_max_m, see below)
      |
[sort by ascending angle]   (canonical order; required for correct circular-neighbor handling)
      |
Outlier Detection      (outliers.detect_outliers)
      |
Noise Filtering         (denoise.median_filter)
      |
Temporal Filtering      (temporal.TemporalFilter -- optional, disabled by default)
      |
PreprocessedScan
      |
(future) Phase 4: Coordinate Transformation
      |
(future) Phase 5: Clustering
```

`ScanFrame` *is* the "RawScan" in this pipeline -- no separate raw-scan type was introduced,
per the project's "do not create duplicate representations of the same data" rule.
`PreprocessedScan` (`models/preprocessing.py`) reuses `LiDARPoint` for retained points, for the
same reason: a clean, retained measurement is still just `(angle, distance, timestamp)`.

**Intentional boundary**: `preprocessing/` imports only from `models` and `common` (both from
the Phase 0 foundation). It is never imported by, and never imports, `simulator`, Unity, the
cloud backend, or any not-yet-built stage (clustering, tracking, collision, ...). Integration
tests that run preprocessing against simulator scenarios live in `simulator/tests/` instead of
`perception/tests/`, specifically so `perception`'s own test suite never needs `simulator`
installed -- see docs/testing.md.

## Validation rules

Implemented in `preprocessing.validation.classify_point`. A measurement is invalid if, in this
order:

1. `LiDARPoint.valid` is already `False` (the data source explicitly flagged it, e.g. a
   simulated "missing measurement" dropout) -- reported as `SENSOR_FLAGGED_INVALID`.
2. `angle` is not finite (NaN/inf) -- `NON_FINITE_ANGLE`.
3. `distance` is not finite (NaN/inf) -- `NON_FINITE_DISTANCE`.
4. `distance <= 0`, or `distance < min_valid_distance_m` -- `BELOW_MIN_VALID_DISTANCE`. The
   near-field **radial** self-return cutoff: a return at (essentially) the sensor origin, plus
   any zero / negative distance regardless of the configured value.
5. The return's `(x, y)` (`x = d*cos(angle)`, `y = d*sin(angle)`) is inside the **ego vehicle
   body rectangle** -- `INSIDE_EGO_FOOTPRINT`. The *geometric* self-return guard: a return
   landing inside the vehicle you are bolted to is a reflection off the body or the sensor mount,
   never a real obstacle. This is what removes a self-return **arc** -- an arc of body points a
   few tens of centimetres out that the radial rule in (4) misses (each point's own distance is
   past the cutoff, but together they form a cluster whose centroid hugs the origin, which then
   becomes a phantom `#track-1` "vehicle" driving CRITICAL risk and `0.00 m` REAR/LEFT clearance).
   See "Ego-vehicle footprint mask" below.
6. `distance < min_range_m` -- `BELOW_MIN_RANGE`.
7. `distance > max_range_m` -- `ABOVE_MAX_RANGE`.

Otherwise the point is valid. Any unexpected exception while classifying a single point (e.g. a
genuinely malformed/corrupted value) is caught and the point is treated as invalid
(`MALFORMED`) rather than crashing the scan -- see "Error handling" below.

**Note on `LiDARPoint`'s own pydantic constraints**: `LiDARPoint.angle` (`ge=0, lt=360`) and
`.distance` (`ge=0`) already reject most malformed values -- including NaN, since `NaN >= 0` is
`False` -- for any point built through normal construction (every current data source does this).
`distance` has no pydantic *upper* bound, so `inf` *can* reach a normally-constructed
`LiDARPoint` -- validation here is what actually catches it. The NaN checks are defense in depth
against any future code path that bypasses model validation (e.g. `model_construct()`, used
directly in `perception/tests/test_preprocessing_validation.py` to exercise this).

## Range filtering

`min_range_m`/`max_range_m` are **not** separate preprocessing parameters -- validation reuses
`common.config.Settings.lidar_range_min_m` / `.lidar_range_max_m`, the same values the simulator
uses to generate scans (Phase 2). There is exactly one configured sensor range in the system.

## Near-field / self-return filtering

`Settings.min_valid_distance_m` (env `LIDAR_MIN_VALID_DISTANCE_M`, default **0.30 m**) is the
minimum distance at which a return counts as a real environmental object. It is a **separate,
larger** threshold than `lidar_range_min_m` (0.05 m): `lidar_range_min_m` is the sensor's raw
lower measurement bound, whereas `min_valid_distance_m` answers "is this the world, or the
vehicle we are bolted to?". Anything closer -- including a zero, negative, or non-finite
distance, regardless of the configured value -- is rejected as `BELOW_MIN_VALID_DISTANCE` and
never reaches the coordinate transform, so it cannot contribute to a cluster, an object or track
count, a classification, a clearance/collision/TTC/prediction figure, or an event-timeline
entry. A track that was already created from such points simply stops being fed and ages out
through the normal tracking lifecycle (`tracking_max_missed_scans` / `tracking_track_timeout_s`);
while it coasts, `clearance.geometry` and `collision.engine` apply the same radial cutoff again
as defense-in-depth so it cannot drive a `0.00 m` clearance, a `0.0 s` TTC, or a CRITICAL risk
from its stale near-origin centroid.

This radial cutoff is a per-point check: it does **not** catch a self-return *arc* whose
individual points sit a few tens of centimetres out. The **ego-vehicle footprint mask** below is
what does.

The default sits at/above the **SLAMTEC RPLIDAR A3M1**'s own 0.20 m rated minimum range
(docs/hardware-validation-procedure.md -- below that the unit cannot range at all), with headroom
for the unreliable near-floor band and the ~0.2 m sensor-mount / ego-body self return a
centre-mounted unit sees on the live rig. It is deliberately *far inside* the ego vehicle's own
0.9 m half-width (`vehicle_width_m` / 2) and far below every configured clearance/collision
threshold (all measured from the safety-envelope edge, >= 1.2 m from the origin), so no
*detectable* environmental object -- which is necessarily at least a sensor-minimum-range away --
is discarded. Retune via `LIDAR_MIN_VALID_DISTANCE_M` for a different sensor or mount (lower for a
bench unit with no vehicle body around it, higher for a bulkier mount).

The classic repro: a moving simulator obstacle (e.g. `08_approaching_obstacle`) whose leading
edge sweeps across the sensor origin makes `Environment.true_distance` return ~0 for a whole
ring of bearings; the simulator's noise model clamps each up to `lidar_range_min_m` (0.05 m),
which passes the `< min_range_m` test, so the ring used to cluster into a phantom `VEHICLE_LIKE`
track sitting at map centre. `min_valid_distance_m` rejects that ring outright.

## Ego-vehicle footprint mask

`common.geometry.EgoFootprint` (env `LIDAR_EGO_FOOTPRINT_FILTER_ENABLED`, default **on**) rejects
any return whose Cartesian `(x, y)` falls inside the ego vehicle's own body rectangle as
`INSIDE_EGO_FOOTPRINT`. It is built **entirely from existing config**: a
`vehicle_length_m` x `vehicle_width_m` rectangle, centred at the vehicle body origin, which sits
at `(-lidar_mount_x_m, -lidar_mount_y_m)` in the sensor frame the perception pipeline works in
(`0deg` = `+x` forward), optionally yawed by `lidar_mount_orientation_deg`, and inflated by
`ego_footprint_margin_m` (default 0.05 m) for sensor-position slop and body protrusions.

Why this and not a bigger `min_valid_distance_m`: the live ESP32 rig's self return is an **arc**
of body/mount reflections roughly 0.3-0.5 m out. Each point's *own* distance clears the radial
`min_valid_distance_m` cutoff, but the arc DBSCAN-clusters into one blob whose centroid hugs the
origin -- reported as `#track-1`, `vehicle_like`, ~0.4 m, driving `RISK: CRITICAL` and
`REAR/LEFT clearance: 0.00 m`. A distance threshold big enough to swallow the whole arc (0.5 m+)
would start deleting genuinely close obstacles and is exactly what "do not blindly increase
`MIN_VALID_DISTANCE_M`" warns against. The footprint mask is **geometric**, not radial: it
removes the vehicle you are bolted to and nothing else. On a bumper-mounted rig
(`lidar_mount_x_m > 0`) the body sits *behind* the sensor, so a real obstacle 0.5 m dead ahead is
outside the rectangle and is kept.

A cleared arc leaves no points near the origin -> no cluster -> no `#track-1` -> honest object /
track count, `SAFE` risk when there is no real hazard, and REAR/LEFT clearance that reads
clear-to-range instead of `0.00 m`. `preprocessing.Preprocessor.process` emits a one-line
`[EGO_MASK]` DEBUG log per scan with the rejected count, so "the sensor is seeing the car" is
visible at a glance rather than leaking downstream.

Disable it (`LIDAR_EGO_FOOTPRINT_FILTER_ENABLED=false`) only for a bench sensor with no vehicle
around it. The radial `min_valid_distance_m` guard still runs independently either way.

## Outlier detection algorithm

Implemented in `preprocessing.outliers.detect_outliers`. A **Hampel-style local-median
identifier**: for each point (in angle-sorted, circular order), take the
`outlier_window_size` measurements angularly nearest to it (including itself, wrapping at
0/360 -- see `preprocessing.windowing.circular_window`), compute their median distance, and flag
the point as an outlier if it deviates from that local median by more than
`outlier_threshold_m`.

**Why this method**: it implements the spec's definition directly -- "isolated values that are
significantly different from their local surroundings" -- and is edge-preserving almost for
free: a window is dominated by whichever value is locally in the majority, so a genuine step
edge (several consecutive points at a new, real distance) shifts the local median *with* the
edge, while a single isolated spike stays a clear minority in its own window and gets flagged.
Verified against the spec's two worked examples directly
(`perception/tests/test_preprocessing_outliers.py`):

- `5.0, 5.1, 5.0, 11.8, 5.1, 5.0` -> the `11.8` is flagged (1 outlier, all else kept).
- `5.0, 5.0, 5.0, 2.0, 2.0, 2.0` -> nothing is flagged (a real 3-point-per-side boundary).

**0/360 handling**: because detection walks a circular window over an angle-sorted array
(wrapping index `-1` to the last element), `358, 359, 0, 1, 2` are true neighbors regardless of
where the scan happens to start -- explicitly tested in
`perception/tests/test_preprocessing_windowing.py` and
`test_preprocessing_outliers.py::TestCircularBoundary`.

**Limitation** (documented, not hidden): like any median-based filter, a genuine feature
narrower than roughly `outlier_window_size // 2` consecutive points can itself be attenuated or
flagged -- `perception/tests/test_preprocessing_outliers.py::test_narrow_feature_shorter_than_half_window_may_be_attenuated`
demonstrates this directly. Keep the default window (5) small relative to the angular width of
the smallest real object you need to preserve; a narrower window (e.g. 3) trades outlier-rejection
robustness for preserving narrower features.

## Noise filtering algorithm

Implemented in `preprocessing.denoise.median_filter`: the same circular local-window machinery
as outlier detection, but instead of flagging deviations it **replaces** each point's distance
with its local window's median.

**Why a median filter** (the spec's requested baseline, evaluated against moving average and
exponential smoothing): it removes small-amplitude Gaussian jitter about as well as a moving
average, but does not blur genuine step edges the way a mean-based filter does, because a median
straddling a step is pulled to whichever side has the majority of the window rather than being
dragged toward the two sides' average. Exponential smoothing has the same edge-blurring problem
*and* is asymmetric/order-dependent, which is a poor fit for a spatial (angular) filter -- it is
used instead for cross-*scan* temporal smoothing (below), where "previous" has an unambiguous
meaning. Verified directly: the spec's example (`5.01, 4.97, 5.04, 4.98, 5.02` -> ~5.0) and a
sharp-step-edge case (`5,5,5,2,2,2` stays `5,5,5,2,2,2`, not blended toward `3.5`).

`median_filter_window <= 1` disables filtering entirely (explicit no-op).

## Temporal filtering

Optional, **disabled by default**. Implemented in `preprocessing.temporal.TemporalFilter`:

```text
filtered_t = alpha * current_t + (1 - alpha) * previous_filtered_t
```

matched **per angle bin** (rounded to 2 decimal places, tolerating float jitter across scans
while keying to a stable angular sampling grid) so a measurement is only smoothed against its
own history, never its neighbors' -- that's the spatial median filter's job. An angle seen for
the first time passes through unfiltered (nothing to blend with yet); an angle bin absent from a
given scan simply keeps its last stored value untouched until it reappears.

**Why disabled by default**: it trades responsiveness for smoothness. For a constant-velocity
ramp (a reasonable local model of an approaching/crossing obstacle), the filter's steady-state
lag is `(1 - alpha) / alpha * step_per_scan` -- for the default `alpha=0.5` and
`08_approaching_obstacle`'s ~0.2m/scan closing rate, that's a **0.2m** steady lag, verified
exactly in `perception/tests/test_preprocessing_temporal.py::TestMovingObstacleLag` (including
that higher `alpha` reduces it further). That's a deliberate, bounded, and now-measured
trade-off rather than a hidden one -- enable it only if a specific downstream consumer needs
smoother-but-laggier distance readings.

Being stateful (it needs the *previous* filtered value per angle), `TemporalFilter` is owned by
one `Preprocessor` instance and persists across calls to `.process()`. Use one `Preprocessor` per
independent scan stream; call `.reset_temporal_state()` before reusing one for a new stream.

## Configuration

All on `common.config.Settings` (see `.env.example`), all overridable via `LIDAR_`-prefixed
environment variables, all with sensible defaults:

| Setting | Default | Meaning |
|---|---|---|
| `lidar_range_min_m` | `0.05` | Reused from Phase 2. Below this, a measurement is invalid (`BELOW_MIN_RANGE`). |
| `lidar_range_max_m` | `12.0` | Reused from Phase 2. Above this, a measurement is invalid. |
| `min_valid_distance_m` | `0.30` | Near-field **radial** self-return cutoff. A distance `<= 0`, non-finite, or below this is rejected as `BELOW_MIN_VALID_DISTANCE` before clustering/tracking (and re-checked in clearance/collision). See "Near-field / self-return filtering". |
| `ego_footprint_filter_enabled` | `True` | Reject returns whose `(x, y)` is inside the ego vehicle body rectangle (`INSIDE_EGO_FOOTPRINT`) -- the **geometric** self-return guard that removes a self-return arc. See "Ego-vehicle footprint mask". |
| `ego_footprint_margin_m` | `0.05` | Inflates the ego body rectangle by this much (metres) for sensor-position slop / body protrusions. |
| `preprocessing_outlier_threshold_m` | `0.5` | Max allowed deviation (meters) from a point's local median before it's flagged as an outlier. |
| `preprocessing_outlier_window_size` | `5` | Approx. number of angular neighbors (incl. self) compared against. `<3` disables outlier detection. |
| `preprocessing_median_filter_window` | `5` | Approx. number of angular neighbors (incl. self) averaged via median. `<=1` disables noise filtering. |
| `preprocessing_temporal_filter_enabled` | `False` | Opt-in cross-scan exponential smoothing. |
| `preprocessing_temporal_filter_alpha` | `0.5` | Weight on the current measurement, `(0, 1]`. Lower = smoother but laggier. |

To change them: set the corresponding `LIDAR_*` environment variable (or edit `.env`), e.g.
`LIDAR_PREPROCESSING_OUTLIER_THRESHOLD_M=0.3`, or pass an explicit
`PreprocessingConfig`/`Settings` instance directly to `Preprocessor(...)` in code (useful for
tests, where a fixed config avoids depending on ambient environment state).

## Scan-quality metrics

`PreprocessedScan.quality_statistics` (`models.preprocessing.ScanQualityStatistics`), computed
by `preprocessing.quality.compute_quality_statistics`:

- `valid_percentage`, `invalid_percentage`, `outlier_percentage` -- relative to `total_count`;
  all `0.0` (not a division error) for an empty scan.
- `mean_distance`, `median_distance`, `minimum_distance`, `maximum_distance` -- computed over the
  final retained `points` (post-outlier-removal, post-noise-filtering); all `None` (not `0.0`,
  which would look like a real close-range reading) when there are no retained points.

These are exactly the numbers a future sensor-health monitor, cloud dashboard, and data-quality
alerting system (Phases 12/14/15/24) will consume -- computed now so those phases only need to
transport and display them, not invent them.

## Count semantics

- `total_count`: measurements in the source `ScanFrame`.
- `invalid_count`: failed validation (see above).
- `valid_count`: `total_count - invalid_count` (passed validation; may still include points
  later removed as outliers).
- `outlier_count`: of the valid measurements, how many were removed as local outliers.
- `len(PreprocessedScan.points) == valid_count - outlier_count`.

## Limitations

- **Circular-neighbor approximation for sparse/gappy scans**: outlier detection and the median
  filter treat "nearest by array index in the angle-sorted list" as "nearest by angle." For a
  complete, evenly-sampled scan (the normal case) these are identical. If many points are
  missing/invalid, the surviving points' index-neighbors may be angularly farther apart than a
  literal fixed-degree window would imply -- a documented, graceful degradation, not a crash.
- **Narrow real features**: see the outlier-detection limitation above -- features wider than
  roughly `window_size // 2` are safe; narrower ones can be attenuated.
- **No Cartesian conversion**: `PreprocessedScan.points` are `LiDARPoint`s (angle, distance,
  timestamp only), by design -- Phase 4 owns polar-to-Cartesian conversion.
- **Single timestamp per scan**: inherited from `ScanFrame`; a full sweep is still treated as
  instantaneous (same simplification noted in docs/simulation.md).
- **Temporal filter angle-bin matching assumes a stable angular sampling grid** across
  consecutive scans from the same source (true for the simulator and for a fixed-resolution real
  sensor). A data source that changes its angular resolution between scans would fragment the
  temporal filter's history unnecessarily (each new resolution effectively starts fresh).

## Performance

Measured with `scripts/benchmark_preprocessing.py` (`perf_counter`-timed `Preprocessor.process()`
calls, 300 scans per scenario, default 360-point/scan scenarios, on the development machine used
for this phase):

| Scenario | avg (ms) | max (ms) | min (ms) |
|---|---|---|---|
| `01_empty` | 3.33 | 11.32 | 2.21 |
| `02_wall_in_front` | 3.08 | 13.92 | 2.18 |
| `05_multiple_obstacles` | 3.09 | 12.03 | 2.22 |
| `09_noisy_lidar` | 2.97 | 10.57 | 2.19 |
| `10_missing_outliers` | 2.70 | 11.42 | 1.92 |

Overall average ~3.0ms/scan, worst observed single-scan time ~13.9ms. Target was 10+ scans/second
at ~360 points/scan (i.e. well under 100ms/scan); this pipeline comfortably supports **300+
scans/second on average**, over an order of magnitude of headroom, even accounting for the
occasional slower call (worst case still supports ~70+ scans/second). No algorithmic step here
is worse than O(n log n) per scan (sorting) with small-constant O(window_size) work per point.

## Example usage

```python
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source

preprocessor = Preprocessor()  # reads defaults from common.config.Settings
source = make_data_source("09_noisy_lidar")

with source:
    raw_scan = source.read_scan()
    clean_scan = preprocessor.process(raw_scan)

print(clean_scan.total_count, clean_scan.valid_count, clean_scan.outlier_count)
print(clean_scan.quality_statistics.mean_distance)
```

For a one-off scan with no temporal-filter state to preserve:
`from preprocessing import preprocess_scan; preprocess_scan(raw_scan)`.

CLI/debug tooling:

```bash
python scripts/compare_raw_processed.py --scenario 10_missing_outliers --scans 3
python scripts/compare_raw_processed.py --scenario 02_wall_in_front --visualize
python scripts/benchmark_preprocessing.py
```
