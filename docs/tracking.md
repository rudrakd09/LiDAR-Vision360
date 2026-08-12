# Object Tracking

## Status

**Implemented (Phase 7)**, in `perception/src/tracking/`. Builds on Phase 0 (Foundation),
Phase 3 (Preprocessing), Phase 4 (Coordinate Transformation), Phase 5 (Obstacle Clustering), and
Phase 6 (Geometric Object Classification) -- all complete and unmodified by this phase. Mapping
through hardware integration (Phases 8-10, 16) are not implemented yet.

## Why tracking is required

Every earlier stage treats each scan independently: `objects.GeometricClassifier` produces a
fresh `ClassifiedScan` from scratch every time, with no memory of what was seen a moment ago. That
is enough to answer "what is out there right now," but not "is the object at (4.8, 2.1) this scan
the same physical thing that was at (5.0, 2.0) last scan" -- and without that answer, nothing
downstream can report a persistent identity, a velocity, a direction, or a predicted future
position. Collision risk / time-to-collision (Phase 9) and Unity trajectory visualization
(Phase 11) both fundamentally need exactly that: not just "there is an obstacle here," but "this
specific obstacle is moving this way, this fast, and will likely be there next."

## Architecture

```
ClassifiedScan
     |
for each existing track: Kalman predict() by one nominal scan interval
     |
Object Association          (tracking.association.associate -- nearest-neighbour, distance-gated)
     |
Track Management             (tracking.track.Track -- lifecycle state machine)
     |
Kalman Filter                (tracking.kalman.KalmanFilter2D -- per-track, [x, y, vx, vy])
     |
TrackedScan
```

`perception/src/tracking/`:

| Module | Responsibility |
|---|---|
| `kalman.py` | `KalmanFilter2D` -- the constant-velocity Kalman filter itself, no project-specific model dependencies. |
| `track.py` | `Track` -- one persistent track: its `KalmanFilter2D` plus lifecycle bookkeeping (age/hits/misses/state) and the most recent detection's classification/shape data. Internal, never exposed outside `tracking`. |
| `association.py` | `associate()` -- greedy nearest-neighbour matching between existing tracks and this scan's detections. |
| `tracker.py` | `ObjectTracker` -- orchestrates predict → associate → update/miss → create/expire → project, once per scan. The only public entry point. |
| `metrics.py` | Generic tracking-quality metrics (no `simulator` dependency) -- see "Tracking quality metrics" below. |
| `visualize.py` | Debug-only matplotlib plot of a `TrackedScan`. |

This module has no dependency on `simulator` or any later pipeline stage (mapping, collision,
clearance, Unity, cloud) -- the same architecture boundary every prior phase established (see
docs/architecture.md). Unlike every earlier stage's stateless `*Clusterer`/`*Classifier`/
`*Transformer`, `ObjectTracker` is inherently **stateful** -- it *is* the tracking session. One
instance must be constructed once and reused across every scan in a stream; a fresh instance per
scan would create a brand-new track for every detection, every time, defeating the entire point.

## Track data model

Per docs/data-model.md "Extensibility rule," a tracked object is not a new concept -- it is still
a `DetectedObject`, now with its Phase-7-reserved fields (`track_id`, `velocity`, `direction`,
already defined since Phase 0) and a handful of additive `Optional` ones defined alongside them in
`models/objects.py` populated:

| Field | Type | Meaning |
|---|---|---|
| `track_id` | `str` | Persistent identifier, unique for the life of the `ObjectTracker` session (e.g. `"track-7"`). |
| `centroid` | `Point2D` | The Kalman filter's *corrected* position estimate (`kf.x`, `kf.y`) -- not the raw detection centroid; see "Kalman filter" below. |
| `velocity` | `Velocity2D \| None` | `None` until `tracking_min_observations_for_velocity` real detections have been applied -- see "Velocity estimation." Has a `.speed` property. |
| `direction` | `float \| None` | Heading in degrees `[0, 360)`, CCW from `+x`, `None` whenever `velocity` is `None` or the estimated speed is ~0. |
| `predicted_position` | `Point2D \| None` | The Kalman filter's one-nominal-scan-ahead position estimate -- see "Prediction." |
| `tracking_state` | `TrackingState \| None` | `TENTATIVE` / `CONFIRMED` / `COASTING` / `LOST` -- see "Track lifecycle." |
| `movement_state` | `MovementState \| None` | `STATIONARY` / `MOVING` / `UNKNOWN` -- see "Movement classification." |
| `track_age` | `int \| None` | Scans since this track was created (including this one). |
| `track_hits` | `int \| None` | Scans in which this track was matched to a real detection (including this one, if matched). |
| `track_misses` | `int \| None` | Current *consecutive* miss streak; `0` while actively detected. |

Named `track_age`/`track_hits`/`track_misses` rather than the spec's `age`/`hits`/`misses` to
avoid ambiguity with any other, non-tracking notion of "age" a future phase might introduce on
`DetectedObject` -- a documented, deliberate naming choice, in the same spirit as `ObstacleCluster`
's `width`/`depth` axis-convention note (docs/clustering.md).

**No new "tracked object" model was introduced.** The scan-level output *is* new --
`models.tracking.TrackedScan` -- following the same per-stage-model pattern `PreprocessedScan` /
`CartesianScan` / `ClusteredScan` / `ClassifiedScan` established, with `objects: list[DetectedObject]`
(one per live track), `noise_points` (passthrough), `object_count`, `noise_count`,
`new_track_count`, `lost_track_count`, `coasting_track_count`.

## Association algorithm

Nearest-neighbour, greedy, not the Hungarian/optimal assignment algorithm -- a deliberate, simple
first cut per this phase's spec ("initially implement a simple robust method"). For every existing
track (already Kalman-`predict()`-ed to this scan) and every detection in the incoming
`ClassifiedScan`, `association._association_cost()` computes:

1. **Hard gate:** Euclidean distance between the track's predicted `(x, y)` and the detection's
   centroid. Pairs beyond `tracking_max_association_distance_m` (default `2.0`) are infeasible --
   excluded entirely, never considered a candidate match.
2. **Soft dimension term:** `tracking_dimension_cost_weight * min(|Δwidth| + |Δdepth|, tracking_max_dimension_diff_m)`
   added to the distance cost. A cluster's width/depth estimate is noisy scan-to-scan (DBSCAN
   membership at a cluster's edge can flicker); this nudges the decision toward a
   similarly-sized candidate on a near-tie, but never excludes a pair outright, and is capped so
   one wildly-misestimated cluster can't dominate the distance term.
3. **Soft classification term:** `tracking_classification_mismatch_penalty` (default `0.5`) added
   if the track's and the candidate detection's classifications differ *and neither is
   `UNKNOWN`*. Per this phase's spec, **classification must never be mandatory for
   association** -- it can legitimately flip frame-to-frame near
   `objects.classification_min_confidence` (docs/object-classification.md), and a track must not
   fragment just because a borderline cluster's label wobbled. `07_moving_crossing`'s fast, small
   pole cluster does exactly this in practice -- see "Scenario results" below.

Every feasible `(track, detection)` pair is then considered in ascending-cost order, and greedily
assigned if both sides are still free -- each track and each detection matches at most once.
Unmatched tracks are handed to `Track.mark_missed()`; unmatched detections spawn new tracks (see
below).

**Known limitation:** greedy assignment is not globally optimal (unlike the Hungarian algorithm,
`scipy.optimize.linear_sum_assignment` -- already available transitively via `scikit-learn`, not
currently used). In a dense scene with several similarly-spaced candidates, greedy can occasionally
pick a locally-cheapest pair that blocks a better global assignment. Acceptable for this phase's
target scenarios (a handful of well-separated obstacles); a documented candidate for a future
upgrade if dense-scene tracking becomes a requirement.

## Track lifecycle

```
                 hits >= tracking_min_hits_to_confirm
   TENTATIVE ─────────────────────────────────────────► CONFIRMED
       │                                                    │  ▲
       │ mark_missed()                     mark_missed() │  │ update()
       ▼                                                    ▼  │
   COASTING/LOST ◄────────────────────────────────────── COASTING
       │            misses > tracking_max_missed_scans
       │            OR (now - last_detection) > tracking_track_timeout_s
       ▼
     LOST  (terminal -- dropped from the active track set)
```

- **TENTATIVE**: just created. A single-scan spurious cluster (sensor noise, a transient
  clustering artifact) must not be reported as a confident, persistent object -- `TrackingState`
  only advances to `CONFIRMED` once `tracking_min_hits_to_confirm` (default `3`) real detections
  have landed on it, including its creating one.
- **CONFIRMED**: a trusted, stable track. Stays `CONFIRMED` on every further real update.
- **COASTING**: missed this scan (no detection associated), but still within the miss budget --
  reported with a Kalman-predicted-only position (no fresh measurement backing it). Reappearing
  from `COASTING` returns straight to `CONFIRMED` (not back through `TENTATIVE`) -- the track
  already proved itself before the gap.
- **LOST**: terminal. A track transitions here the scan its miss budget is exceeded, and is
  dropped from `TrackedScan.objects` that same scan (counted in `lost_track_count` instead, never
  silently vanished). `LOST` track IDs are never reused -- a physical object that reappears after
  timing out gets a *new* track ID, by design (see "Known limitations").

Every track (TENTATIVE, CONFIRMED, or COASTING alike) uses the same uniform miss-tolerance
policy -- deliberately not a stricter rule for TENTATIVE tracks, for simplicity in this baseline;
see "Known limitations." The miss budget itself is two independent knobs, either crossing its
threshold is sufficient:

- `tracking_max_missed_scans` (default `5`): a scan-count budget, robust to variable scan rate.
- `tracking_track_timeout_s` (default `1.0`): a wall-clock backstop.

This handles brief LiDAR occlusion or missed detections (temporary occlusion, a gap in the
simulator's ray-cast) without discarding the track's identity or velocity history -- see
"Scenario results" for a worked example.

## Kalman filter

`KalmanFilter2D` (`tracking/kalman.py`) is a standard constant-velocity (CV) Kalman filter, one
per track, entirely independent of `models`/`simulator`.

**State vector:** `[x, y, vx, vy]^T` -- position and velocity, meters and meters/second. The only
thing ever *measured* is position (a cluster's centroid, from `objects.classifier`); velocity is
purely inferred by the filter from how position changes across updates. There is no direct
velocity sensor.

**Motion model** (state-transition matrix `F`, applied over `dt` seconds):

```
x' = x + vx*dt      |1  0  dt  0|
y' = y + vy*dt   F = |0  1  0   dt|
vx' = vx             |0  0  1   0|
vy' = vy             |0  0  0   1|
```

**Process noise** (`Q`): the discrete white-noise-acceleration (DWNA) model -- an assumed
unmodeled acceleration of standard deviation `tracking_process_noise_std_mps2` (default
`0.5 m/s²`) drives one independent noise block per axis:

```
Q_axis = q * | dt⁴/4  dt³/2 |     q = tracking_process_noise_std_mps2²
             | dt³/2  dt²   |
```

interleaved into the full 4x4 `Q` per-axis (x/vx and y/vy independently). This is what lets the
filter track a real turn or a speed change rather than being locked to whatever velocity it first
inferred.

**Measurement model:** `H = [[1,0,0,0],[0,1,0,0]]` (position only), with measurement noise
`R = diag(tracking_measurement_noise_std_m², tracking_measurement_noise_std_m²)`
(default `0.15 m`).

**Prediction step** (`predict(dt)`): `state = F @ state`, `P = F @ P @ Fᵀ + Q`. Mutates the
filter -- called once per scan, per active track, before association.

**Measurement-update step** (`update(x, y)`): the standard Kalman correction --
`innovation = z - H @ state`, `S = H @ P @ Hᵀ + R`, `K = P @ Hᵀ @ S⁻¹`,
`state += K @ innovation`, `P = (I - K @ H) @ P`. Mutates the filter -- called once per scan, only
for tracks matched to a real detection.

**Initial uncertainty:** `P` starts as `diag(σ_pos², σ_pos², σ_vel², σ_vel²)`, where `σ_pos` =
`tracking_initial_position_uncertainty_m` (default `1.0m`) and `σ_vel` =
`tracking_initial_velocity_uncertainty_mps` (default `5.0 m/s`) -- position is fairly well known
at creation (it *is* the creating detection), velocity is not known at all yet, hence the much
larger initial velocity uncertainty.

None of the above is hard-coded: every noise/uncertainty value is a `common.config.Settings`
field (`tracking_*`), read once at `Track` construction.

### Time step (dt) handling

Every `predict()` call advances the filter by one **fixed nominal** scan interval
(`1 / lidar_scan_frequency_hz`), *not* the measured delta between consecutive `ScanFrame.timestamp`
wall-clock values. This was a deliberate fix, not the first thing tried: using measured
`scan.timestamp` deltas was implemented first, and measured to inflate estimated velocity by
roughly 8x (a true 2.0 m/s approach in `08_approaching_obstacle` came out around -16 m/s) --
because `simulator.datasource.SimulatedLiDARDataSource`, in its default (non-real-time,
batch/test) mode, produces consecutive scans as fast as Python can run: `ScanFrame.timestamp`
deltas of a few milliseconds, while the *simulated world* between those same two scans always
advances by the model's nominal `1 / scan_frequency_hz` (~100ms at the default 10Hz) --
independent of wall-clock pacing, by that module's own explicit, pre-existing design principle
("obstacle motion between scans always advances by the model's nominal
`1 / scan_frequency_hz` ... deterministic and independent of wall-clock jitter"). Using nominal dt
for the Kalman step mirrors that exact same principle rather than fighting it, and fixed the
inflated-velocity bug immediately (confirmed convergence to the true ±0.01 m/s on both moving
scenarios -- see "Scenario results").

Reuses `lidar_scan_frequency_hz` rather than introducing a new setting -- there is exactly one
configured scan rate in the system, the same reuse precedent `preprocessing_*` already established
for `lidar_range_min_m`/`lidar_range_max_m` (see `common/config.py`).

**Known limitation:** this assumes a constant, known scan rate. Real hardware (Phase 16) with
genuine timing jitter or dropped frames would need measured elapsed time instead -- reintroducing
wall-clock `dt`, but this time validated against real timestamps rather than a fast-forwarding
simulator, is flagged as a specific follow-up for that phase.

## Velocity estimation

`velocity` stays `None` on the reported `DetectedObject` until a track has accumulated
`tracking_min_observations_for_velocity` (default `3`) real (non-coasting) measurement updates --
using `track_hits` directly, since that is already exactly "how many real detections this track
has ever had." A 1-2-point velocity estimate off a just-created track is dominated by measurement
noise, not real motion; reporting it anyway would be actively misleading rather than merely
imprecise. Once reliable:

```
speed = sqrt(vx² + vy²)                    (Velocity2D.speed property, models/objects.py)
direction = atan2(vy, vx) in degrees, [0, 360)   (None if speed ~ 0)
```

## Movement classification

`movement_state` is `UNKNOWN` until velocity is reliable (see above); once reliable:

- `STATIONARY` if `speed < tracking_stationary_speed_threshold_mps` (default `0.3 m/s`)
- `MOVING` otherwise

The threshold is set above the residual velocity "noise floor" a perfectly stationary object's
Kalman-filtered estimate still shows -- driven by `tracking_measurement_noise_std_m` (0.15m) and
DBSCAN cluster-membership jitter at a cluster's boundary shifting the centroid slightly
scan-to-scan, both of which a finite-difference-style velocity estimate amplifies -- and well
below both moving-obstacle scenarios' actual ground-truth speeds (`07_moving_crossing`: 1.5 m/s,
`08_approaching_obstacle`: 2.0 m/s), so there is a wide, safe margin on both sides. Confirmed
empirically: every stationary scenario (`02_wall_in_front`, `03_pole_left`, `04_vehicle_ahead`)
settles to `STATIONARY` with estimated speed well under the threshold once velocity becomes
reliable (see "Scenario results").

## Prediction

`predicted_position` is the Kalman filter's constant-velocity extrapolation one nominal scan
interval past its *current, already-corrected* state -- computed by `KalmanFilter2D.peek_predict()`,
which (unlike `predict()`) does **not** mutate the filter's `state`/`P`; it is a read-only "what
happens next" query, distinct from the actual `predict()` step that advances the filter for the
next scan's association/update cycle. This is what a future collision/TTC engine (Phase 9) and
Unity trajectory visualization (Phase 11) will consume -- not implemented in this phase, per its
explicit scope (see PROJECT_SPECIFICATION.md).

## Configuration

All in `common.config.Settings` (`perception/src/common/config.py`), `LIDAR_`-prefixed env-var
overridable, nothing hard-coded:

| Setting | Default | Meaning |
|---|---|---|
| `tracking_max_association_distance_m` | `2.0` | Hard gate: max distance between a track's predicted position and a candidate detection. |
| `tracking_dimension_cost_weight` | `0.3` | Soft cost weight on width/depth difference. |
| `tracking_max_dimension_diff_m` | `1.5` | Cap on the dimension term's contribution. |
| `tracking_classification_mismatch_penalty` | `0.5` | Soft cost added on a classification mismatch (never a hard exclusion). |
| `tracking_min_hits_to_confirm` | `3` | Real detections needed for TENTATIVE → CONFIRMED. |
| `tracking_max_missed_scans` | `5` | Consecutive misses tolerated before LOST. |
| `tracking_track_timeout_s` | `1.0` | Wall-clock backstop, alternative trigger for LOST. |
| `tracking_process_noise_std_mps2` | `0.5` | Kalman process noise (assumed acceleration std, m/s²). |
| `tracking_measurement_noise_std_m` | `0.15` | Kalman measurement noise (position std, m). |
| `tracking_initial_position_uncertainty_m` | `1.0` | Kalman initial position std. |
| `tracking_initial_velocity_uncertainty_mps` | `5.0` | Kalman initial velocity std. |
| `tracking_min_observations_for_velocity` | `3` | Real detections needed before velocity is reported. |
| `tracking_stationary_speed_threshold_mps` | `0.3` | STATIONARY / MOVING boundary. |

`tracking_measurement_noise_std_m` (0.15m) was set from this project's own numbers, not picked
arbitrarily: it reflects observed DBSCAN-centroid jitter scan-to-scan (dominated by which points
fall in/out of a cluster at its boundary as an obstacle moves or the scan angle samples shift),
which empirically dominates the raw per-point sensor noise (`lidar_distance_noise_std_m` = 0.02m,
averaged down across a whole cluster's points). `tracking_max_association_distance_m` (2.0m) was
checked against both moving scenarios' true per-scan displacement at the default 10Hz rate
(`07_moving_crossing`: 1.5 m/s → 0.15m/scan; `08_approaching_obstacle`: 2.0 m/s → 0.2m/scan) --
comfortably inside the gate even allowing for prediction error and a missed scan or two, while
still well below the ~0.6m `clustering_eps_m` gap that keeps genuinely distinct simulator
obstacles from ever appearing this close together in the first place.

## Tracking quality metrics

`tracking.metrics` (no `simulator` dependency, mirroring `objects.metrics`'s own separation for
classification):

- `track_summary(scans)` -- total tracks ever created, active tracks at the end, total tracks
  lost, and mean/max track age/hits/misses over the final scan's active tracks.
- `track_id_consistency(id_sequences)` -- fraction of consecutive-scan steps, across one or more
  per-object track-ID sequences, that kept the same ID (`1.0` = never reassigned/lost).
- `position_error(estimated, ground_truth)` / `velocity_error(estimated, ground_truth)` -- mean/max
  Euclidean error between parallel `(x, y)` / `(vx, vy)` sequences, given ground truth.

Ground-truth-aware evaluation against the simulator's known obstacle motion lives in
`scripts/evaluate_tracking.py` (depends on both `simulator` and `perception`, same separation
`scripts/evaluate_classification.py` established) -- see "Scenario results" below for its output.

## Test visualization

`tracking.visualize.plot_tracked_scan()` (matplotlib, optional `viz` extra): each tracked object's
current position (marker shape encodes `tracking_state` -- circle/star/triangle/x for
tentative/confirmed/coasting/lost), a velocity vector arrow (only once reliable), a faint marker
at the predicted next position, and a `#track_id classification (state)` + speed text label; noise
points shown separately. Debugging/algorithm-validation only -- this is what will later map onto
Unity's trajectory visualization (Phase 11), not a replacement for it.

```
python scripts/visualize_tracking.py --scenario 08_approaching_obstacle --scans 15 --explain
python scripts/visualize_tracking.py --scenario 07_moving_crossing --scans 20 --visualize
```

`--explain` prints each track's full state in the spec's example format:

```
Object #track-1
  vehicle_like
  Distance: 4.95 m
  Speed: 2.01 m/s
  Direction: toward vehicle
  Tracking state: confirmed  age=15 hits=15 misses=0
  Predicted next position: (4.75, -0.00)
```

("Direction: toward vehicle" is presentation logic in the script only, derived from the cosine
between the velocity vector and the vector back to the vehicle origin -- not a field stored on the
model itself.)

## Scenario results

From `scripts/evaluate_tracking.py` (20 scans each, first 3 excluded as a confirm/velocity-settle
warmup):

| Scenario | True velocity | Estimated velocity (final) | Track ID consistency | Position error (mean/max) | Velocity error (mean/max) |
|---|---|---|---|---|---|
| `07_moving_crossing` | `(0.0, 1.5) m/s` | `(≈0.00, ≈1.49) m/s` | `1.000` | `0.178m` / `0.198m` | `0.078` / `0.407 m/s` |
| `08_approaching_obstacle` | `(-2.0, 0.0) m/s` | `(-2.00, ≈0.00) m/s` | `1.000` | `2.246m` / `2.251m`* | `0.006` / `0.059 m/s` |

\* `08_approaching_obstacle`'s position error is a known, expected geometric artifact, **not**
tracking inaccuracy: ground truth here is the rectangle obstacle's *geometric center*, but a 2D
LiDAR only ever sees a surface -- the tracked centroid is the rectangle's *near face*, offset from
its center by a constant ~2.25m (half its 4.5m depth). Velocity error, unaffected by a constant
offset, is the more meaningful accuracy signal for extended (non-point) obstacles and confirms the
filter is accurate (`0.006 m/s` mean error against a `2.0 m/s` true speed). See
`scripts/evaluate_tracking.py`'s docstring for the full explanation.

Both scenarios: a single, stable `track-1` for the entire 20-scan run (`track_id_consistency =
1.0`) -- including `07_moving_crossing`, whose small, fast pole cluster's *classification*
genuinely flickers between `POLE_LIKE` and `UNKNOWN` scan-to-scan (expected, documented in
docs/object-classification.md), confirming association correctly treats classification as
non-mandatory rather than fragmenting the track on every flicker.

Stationary scenarios (`02_wall_in_front`, `03_pole_left`, `04_vehicle_ahead`): a single stable
track per scenario, `movement_state` settles to `STATIONARY` with estimated speed well under
`tracking_stationary_speed_threshold_mps` once velocity becomes reliable, and zero tracks lost
across 10 scans.

`01_empty`: zero tracks created, ever. `06_narrow_corridor`: two distinct, stable tracks (one per
wall), zero lost.

## Performance

`scripts/benchmark_tracking.py`, 200-300 scans per scenario:

| Scenario | Tracking-only avg | Tracking-only max | Full-chain avg | Full-chain scans/sec |
|---|---|---|---|---|
| `01_empty` | `0.04ms` | `0.12ms` | `5.5ms` | `183` |
| `05_multiple_obstacles` | `0.66ms` | `2.37ms` | `10.3ms` | `97` |
| `07_moving_crossing` | `0.16-0.22ms` | `0.62-0.67ms` | `14.7-23.0ms` | `43-68` |
| `08_approaching_obstacle` | `0.20ms` | `0.50ms` | `9.2ms` | `108` |
| `09_noisy_lidar` | `0.49ms` | `1.33ms` | `10.0ms` | `100` |

The tracking stage itself is consistently sub-millisecond -- predict + associate + update for a
handful of tracks is cheap. "Full-chain" includes every earlier stage (preprocessing, coordinates,
clustering, classification) too, and one run showed a single ~1.7s outlier scan in
`07_moving_crossing`; isolating each stage confirmed this spike originates in Phase 5's
`clustering` (sklearn `DBSCAN`), reproducible in `scripts/benchmark_clustering.py` alone --
pre-existing, unrelated to and not introduced by this phase, not investigated further here as it
is out of Phase 7's scope. Even counting that outlier, average full-chain throughput
(43-183 scans/sec depending on scenario) stays comfortably above the real-time target (10+
scans/sec at the default 10Hz configured scan rate).

## Example usage

```python
from clustering import DBSCANClusterer
from coordinates import CoordinateTransformer
from objects import GeometricClassifier
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from tracking import ObjectTracker

preprocessor, transformer, clusterer, classifier = Preprocessor(), CoordinateTransformer(), DBSCANClusterer(), GeometricClassifier()
tracker = ObjectTracker()  # MUST be constructed once and reused across scans
source = make_data_source("08_approaching_obstacle")

with source:
    for _ in range(20):
        raw_scan = source.read_scan()
        clean_scan = preprocessor.process(raw_scan)
        cartesian_scan = transformer.transform(clean_scan)
        clustered_scan = clusterer.cluster(cartesian_scan)
        classified_scan = classifier.classify(clustered_scan)
        tracked_scan = tracker.update(classified_scan)

for obj in tracked_scan.objects:
    speed = obj.velocity.speed if obj.velocity else None
    print(obj.track_id, obj.classification.value, obj.tracking_state.value, speed, obj.direction)
```

## Known limitations

- **Greedy, not optimal, association.** See "Association algorithm" above -- a documented,
  intentional first cut; `scipy.optimize.linear_sum_assignment` (Hungarian algorithm) is a
  concrete future upgrade if dense-scene tracking ever needs it.
- **Nominal, not measured, time step.** See "Time step handling" above -- correct against this
  project's simulator (and any fixed-rate real sensor), but real hardware with genuine timing
  jitter would need this revisited (flagged for Phase 16).
- **Uniform miss-tolerance policy.** A `TENTATIVE` track that misses immediately after creation
  gets the same `tracking_max_missed_scans`/`tracking_track_timeout_s` budget as a long-lived
  `CONFIRMED` one, rather than a stricter (faster-to-drop) rule -- simpler for this baseline, at
  the cost of tolerating a spurious single-scan detection's "track" slightly longer than strictly
  necessary before it naturally times out unconfirmed.
- **`LOST` track IDs are never reused.** A physical object that reappears after exceeding the miss
  budget is, by design, a *new* track with a new ID -- there is no re-identification/re-acquisition
  logic (e.g. matching against a recently-lost track's last known position/velocity/shape). This
  is the correct, honest behavior for this phase (a truly lost track's future position is not
  reliably predictable arbitrarily far out), but means a long occlusion (longer than the miss
  budget) is reported as "object disappeared, new object appeared" rather than "object
  reacquired."
- **2D LiDAR surface-only ground truth mismatch.** See the `08_approaching_obstacle` position-error
  footnote above -- ground truth evaluation compares against geometric obstacle centers, not the
  LiDAR-visible near-surface a 2D scanner can actually measure. A known, explained artifact of the
  evaluation script, not the tracker.
- **No re-identification across a classification change to a *different specific* category
  outside association's soft penalty range**, e.g. a wall fragment that starts scoring confidently
  as `VEHICLE_LIKE` (a documented Phase 6 ambiguity, see docs/object-classification.md "Known
  failure cases") is still tracked correctly by position (classification is soft, not mandatory),
  but its reported `classification` will reflect whatever the classifier said most recently, with
  no cross-checking against tracking history.
