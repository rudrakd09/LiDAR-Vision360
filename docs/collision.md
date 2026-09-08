# Collision & Clearance Engines

## Status

**Both engines implemented.** Collision/risk engine (Phase 9), in `perception/src/collision/`.
Builds on Phase 0 (Foundation) through Phase 8 (2D Occupancy Grid Mapping) -- all complete and
unmodified by this phase. **Clearance engine (Phase 10, `perception/src/clearance/`) is also
implemented** -- see "Directional clearance" below for the full write-up (this section used to be
a stub; the engine now exists, driven by the same demo work that wired it into the streaming
protocol -- see docs/communication.md "Perception frame").

**This is a prototype collision/clearance-awareness system for a 2D-LiDAR-based ground vehicle,
built for simulation and research purposes. It is not a certified automotive safety system, has
not been validated against any regulatory standard (ISO 26262, Euro NCAP, etc.), and must not be
used as the sole basis for a real safety-critical decision.** Every threshold documented below is
a reasoned prototype default, not a validated safety parameter -- see "Parameter selection".

## Objective

Given tracked objects (Phase 7) and the vehicle's own geometry/state, answer: is an object inside
the vehicle's projected path? How far away is it? Is it approaching? What is the estimated
Time-to-Collision? What is the current risk -- SAFE, WARNING, or CRITICAL? Deterministic,
explainable, configurable rule-based logic throughout -- **no machine learning**.

## Architecture

```
TrackedScan
     |
for each tracked DetectedObject:
    relative_motion()          -> relative_position, relative_velocity      (.geometry)
    in_projected_path()        -> bool, purely geometric                    (.geometry)
    compute_ttc()               -> closed-form longitudinal TTC              (.ttc)
    simulate_collision()        -> collision_predicted + time/position       (.prediction)
    assess_risk()                -> risk_level, reason                        (.risk)
    risk_score()                 -> continuous companion metric                (.risk)
     |
CollisionRiskResult
     |
overall_risk = the single highest risk_level among all results
most_critical_object = the result driving it
     |
CollisionAssessment
```

`perception/src/collision/`:

| Module | Responsibility |
|---|---|
| `geometry.py` | Vehicle footprint, frame rotation/translation, relative motion, `in_projected_path`. |
| `ttc.py` | Closed-form longitudinal Time-to-Collision. |
| `prediction.py` | Discrete footprint-intersection simulation over a bounded horizon. |
| `risk.py` | Rule-based SAFE/WARNING/CRITICAL classification + explainable reasons + continuous risk score. |
| `engine.py` | `CollisionRiskEngine` -- the only public entry point. |
| `metrics.py` | Generic ground-truth evaluation (no `simulator` dependency). |
| `visualize.py` | Debug-only matplotlib plot. |

**Primary inputs are tracked-object position/velocity/geometry (Phase 7), vehicle geometry, and
vehicle state -- not the occupancy grid.** Per this phase's own explicit instruction ("do not make
the collision engine dependent exclusively on the occupancy grid"), `collision` has **zero**
dependency on `mapping` (Phase 8) at all; the grid remains available as optional supporting
spatial information for a future phase to layer in without requiring it here. This module has no
dependency on `simulator` or any later pipeline stage (clearance, Unity, cloud) -- the same
architecture boundary every prior phase established.

Unlike `tracking.ObjectTracker`/`mapping.OccupancyGridMapper` (Phase 7/8), `CollisionRiskEngine`
is **stateless** -- every `evaluate()` call is independent, driven entirely by that scan's
tracked objects and the supplied `VehicleState`.

## Vehicle model

Reuses `Settings.vehicle_width_m`/`vehicle_length_m` (reserved since Phase 0, "used from
Phase 9-10 onward" -- this is the first phase that actually consumes them). The vehicle's bare
body is an axis-aligned (in its own heading frame) rectangle centered on `VehicleState.pose`:
half-length `vehicle_length_m / 2` fore and aft, half-width `vehicle_width_m / 2` each side.

Four configurable safety margins (`front_safety_margin_m`, `rear_safety_margin_m`,
`left_safety_margin_m`, `right_safety_margin_m` -- same config section as the vehicle dimensions,
since Phase 10's clearance engine will need the same envelope) expand the body into a **safety
envelope** used for `in_projected_path` and the discrete collision-prediction simulation.
Defaults: front `1.0m` > rear `0.5m` (more buffer is warranted in the vehicle's primary direction
of travel and stopping-distance exposure), left/right `0.3m` each (modest, symmetric lateral
clearance -- e.g. mirror/door swing).

## Safety zones

Not radial distance-from-center rings -- **rectangular, heading-aligned envelopes**, per this
phase's own explicit instruction ("a point 2m directly ahead is very different from 2m to the
side"):

- **Vehicle body**: the bare `vehicle_length_m x vehicle_width_m` rectangle.
- **Safety-margin envelope** ("WARNING zone" in the visualization): the body expanded by the four
  configured margins.
- **SAFE / WARNING / CRITICAL** are not separate static geometric rings at all -- they are the
  *outcome* of `collision.risk.assess_risk`'s rule cascade, which combines geometry (footprint
  overlap, `in_projected_path`) with motion (TTC, closing speed) and configured thresholds. See
  "Risk classification" below for why a purely-geometric-zone model was deliberately not used as
  the final decision.

## Projected path

`collision.geometry.in_projected_path`: **purely geometric**, independent of vehicle speed -- is
the object within the vehicle's heading-aligned corridor, from the rear safety envelope behind to
the sensor's own `lidar_range_max_m` ahead (reused directly -- "there is exactly one configured
sensor range in this project"), laterally within the left/right safety envelope. A stationary
vehicle can and should still know what's directly ahead of it in its lane, which is why this check
never depends on `VehicleState.speed_mps` -- speed/motion is what TTC and collision prediction
(below) are for.

## Relative motion

`collision.geometry.relative_motion(object_position, object_velocity, vehicle_state)` ->
`(relative_position, relative_velocity)`, both reused directly from the existing `models.objects.
Point2D`/`Velocity2D` types (no duplicate flat-field model) -- object minus vehicle, world frame.
`relative_speed` on `CollisionRiskResult` is simply `relative_velocity`'s magnitude.

**Missing object velocity** (`DetectedObject.velocity is None` -- the track is too new for
`tracking.ObjectTracker` to have reported a reliable estimate yet, see docs/tracking.md "Velocity
estimation") is treated as **stationary** `(0, 0)`. This is a documented, but not risk-free,
simplification: `CollisionRiskEngine` appends an explicit caveat to `reason` in this case
("Object velocity not yet reliable ... assumed stationary ... real risk may be higher if it is in
fact moving") rather than silently presenting false certainty.

## TTC calculation

**Deliberately not `distance / speed`.** `collision.ttc.compute_ttc` computes time-to-collision
along the vehicle's **heading axis** -- the only direction the vehicle itself can move in this
project's motion model (`VehicleState.speed_mps` is a single scalar, no independent
lateral/steering velocity is modeled anywhere in this project):

1. Rotate `relative_position`/`relative_velocity` into the vehicle's heading-aligned frame
   (`along`, `lateral`) via `collision.geometry.rotate_to_heading`.
2. `closing_speed = closing_speed_along(...)` (positive = the gap is shrinking). If
   `closing_speed <= collision_minimum_closing_speed_mps` (a noise floor, `0.05 m/s` by default)
   -> **`ttc = None`** (not meaningfully approaching -- covers "moving away", "stationary", and
   noisy sub-threshold velocity). **This gate is checked first**, before proximity: an object
   that is not closing has no "time to collision" regardless of how close it is -- that is a
   distance/clearance concern, and `assess_risk`'s distance rule still escalates it. So a
   stationary obstacle, even one already overlapping the footprint, reports `ttc = None` ("N/A"),
   not `0.0`.
3. `remaining_gap = |along| - vehicle_contact_gap - object_contact_gap`, where
   `vehicle_contact_gap` is the vehicle's own envelope extent on the relevant side (front if the
   object is ahead, rear if behind) and `object_contact_gap` is the object's own along-axis
   half-extent (see "Object footprint approximation" below).
4. If `remaining_gap <= 0`, the object is closing **and** the footprints already overlap along
   this axis -> **`ttc = 0.0`** (contact now).
5. Otherwise, **`ttc = remaining_gap / closing_speed`**.

A centroid within `min_valid_distance_m` (`LIDAR_MIN_VALID_DISTANCE_M`) of the sensor is treated
by `collision.engine` as an ego/self return: `ttc = None`, `risk = SAFE`, excluded from the
overall risk. See "Self-return guard" below.

This is a **1D projection** of the true motion onto the heading axis -- a deliberate
simplification. It correctly, cheaply handles the common case (an object ahead/behind on a
collision course) but says nothing on its own about whether a *laterally-crossing* object will
actually still be in the vehicle's lane at the moment its longitudinal gap closes. That is exactly
what "Collision prediction" below is for -- kept as a **separate, distinct value** on
`CollisionRiskResult` rather than conflated with `ttc`.

**Edge cases**, all directly unit-tested (`perception/tests/test_collision_ttc.py`):
zero relative velocity -> `None`; negative closing velocity (moving away) -> `None`, never a
negative TTC; stationary object already overlapping the footprint -> `None` (not closing);
closing object already overlapping the footprint -> `0.0` (contact now); never a crash; no
division by zero (the minimum-closing-speed gate is checked *before* any division); a genuinely
large TTC (slow approach from far away) is still a real, uncapped number -- only the discrete
simulation
(`collision_predicted`) is horizon-bounded, `ttc` itself is not.

## Collision prediction

`collision.prediction.simulate_collision`: a **discrete footprint-intersection simulation**,
stepping forward in `collision_simulation_step_s` increments (default `0.1s`) up to
`collision_prediction_horizon_s` (default `5.0s`). At each step, both the vehicle (extrapolated
along its heading at `speed_mps`) and the object (extrapolated at its own constant velocity) are
advanced, and the object's position is checked against the vehicle's safety-envelope rectangle
(rotated into the vehicle's own heading frame) expanded by the object's own half-extents.

**Why this is a separate computation from `ttc`, not just its horizon-bounded twin:** this is
the one place in this phase that genuinely, correctly handles a **laterally-crossing object** --
whether an object crossing the vehicle's general direction of travel will actually still be
inside the lateral safety envelope at the moment the longitudinal gap closes. `ttc`'s 1D
projection cannot express this; only a full 2D simulation can. This is exactly what makes
`07_moving_crossing` behave correctly (see "Crossing obstacle" below): `collision_predicted` is
checked **before, and independently of,** the `in_projected_path` gate in `assess_risk` (see
"Risk classification"), since the simulation already incorporates the vehicle's real footprint as
its own intersection target.

Both extrapolations assume constant velocity -- the same short-horizon, "do not claim
centimeter-level accuracy" assumption `tracking.KalmanFilter2D`'s own one-step
`predicted_position` already makes (Phase 7).

## Object footprint approximation

Both TTC and collision prediction need an object "size" to compute a contact gap.
`collision.geometry.object_half_extents(width, depth, settings)` returns
`(half_extent_along, half_extent_lateral)`, using `DetectedObject.depth` (radial/X extent) for
the along-axis half-extent and `.width` (lateral/Y extent) for the lateral one -- **not** a single
`max(width, depth)` applied uniformly to both, which was tried first and found to badly
overestimate a wide-but-shallow object's extent along the closing axis: `04_vehicle_ahead`'s
detected near-face is `~1.7m` wide but only `~0.05m` "deep" as actually observed (a 2D LiDAR only
ever sees an obstacle's near surface, never its far side), and using the `1.7m` width as the
along-axis contact gap put its near edge nearly a meter closer than reality, producing a false
immediate-contact prediction from the very first scan. Both half-extents are floored at
`collision_minimum_object_radius_m` (`0.15m` -- matching the simulator's own smallest modeled
obstacle) against degenerate/tiny detections.

**Documented limitation:** this assumes `width`/`.depth` (fixed at classification time under
`objects.GeometricClassifier`'s own vehicle-at-origin/heading-0 assumption) align with the
vehicle's *own* along/lateral axes -- exactly true under this phase's default identity
`VehicleState` pose, not necessarily true under a non-identity heading (nothing in this project
tracks an object's own orientation independent of the viewing angle it happened to be observed
from).

## Risk classification

`collision.risk.assess_risk`: a most-severe-first rule cascade, the same "score/classify +
human-readable reasons, no black-box weights" approach `objects.scoring` (Phase 6) established
for shape classification, applied here to collision risk.

```
1. collision_predicted and predicted_collision_time <= critical_ttc  -> CRITICAL
2. collision_predicted and predicted_collision_time <= warning_ttc   -> WARNING
   (checked FIRST, un-gated by in_projected_path -- see below)
3. not in_projected_path                                              -> SAFE
4. ttc is not None and ttc <= critical_ttc                            -> CRITICAL
5. not receding and distance <= critical_distance                     -> CRITICAL
6. ttc is not None and ttc <= warning_ttc                              -> WARNING
7. not receding and distance <= warning_distance                      -> WARNING
8. otherwise                                                            -> SAFE
```

**Never proximity alone**: steps 5/7 (the only purely-distance-based checks) are gated by both
`in_projected_path` (an object to the side, however close, never reaches them -- step 3) and "not
confirmed receding" (see below).

**`collision_predicted` is checked first, un-gated by the coarse `in_projected_path` corridor
test.** This was a real bug found during development: an earlier version gated *everything* behind
`in_projected_path`, which meant a laterally-crossing object -- correctly predicted by the 2D
simulation to intersect the vehicle later -- was reported SAFE simply because its *current*
position wasn't yet inside the corridor. Since `collision_predicted` already incorporates the
vehicle's real footprint+margins as its own intersection target (strictly more precise than the
coarse corridor check), it is authoritative on its own; steps 3+ apply only when no future
intersection was found.

**"Do not generate collision warnings simply because an object is close"** (this phase's own
explicit instruction): a **confirmed-receding** object (`closing_speed` clearly negative -- below
`-collision_minimum_closing_speed_mps`, not just "not currently approaching") never triggers the
distance-based checks (5/7) purely for being nearby. This was also found and fixed during
development: distance-based checks originally fired unconditionally on proximity, which flagged a
WARNING for an object at exactly `warning_distance_m` moving directly away. An object with
*ambiguous* (near-zero, within the noise floor) relative motion still gets the distance check --
it might start moving, or the vehicle might.

## Stationary objects

- **Stationary vehicle + stationary object 10m ahead** (this phase's own explicit example): `ttc`
  is `None` (zero relative motion, gated by the closing-speed floor), `collision_predicted` is
  `False` (nothing ever reaches the footprint in the simulation), and `10m > warning_distance_m`
  (`5.0m` default) -> **SAFE**, not falsely CRITICAL.
- **Moving obstacle approaching a stationary vehicle**: `ttc` and `collision_predicted` both
  become well-defined and shrink/trigger as the gap closes, exactly as for a moving-vehicle case
  -- the math only cares about *relative* motion, not which side is doing the moving.
- **Moving vehicle + stationary obstacle**: identical treatment -- `closing_speed_along` is
  computed from the *relative* velocity (object velocity minus vehicle velocity vector), so a
  stationary obstacle ahead of an approaching vehicle produces the same well-defined,
  correctly-shrinking TTC.

## Crossing obstacle (`07_moving_crossing`)

Must only become dangerous if its predicted trajectory actually intersects the vehicle's path --
not simply for being nearby and moving. Verified directly
(`perception/tests/test_collision_prediction.py::TestCrossingObject`,
`simulator/tests/test_collision_integration.py::TestMovingCrossingScenario`):

- Crossing *toward* the lane, with the ego vehicle also closing the longitudinal gap ->
  `collision_predicted = True`, correctly found by the 2D simulation.
- Crossing *away* from the lane (same starting position, opposite lateral velocity) ->
  `collision_predicted = False`, even with the ego vehicle closing the longitudinal gap the exact
  same way -- the simulation correctly distinguishes the two.
- Crossing far ahead with a **stationary ego vehicle** (never closes the longitudinal gap at all)
  -> `collision_predicted = False` regardless of lateral motion -- verified the full scenario
  never reaches CRITICAL and most scans stay SAFE.

## Multiple obstacles

`CollisionRiskEngine.evaluate()` returns one `CollisionRiskResult` per tracked object plus
`overall_risk` (the single highest severity among them) and `most_critical_object` (ties broken
by lowest TTC, then shortest distance).

## Collision risk result model

`models.collision.CollisionRiskResult` / `CollisionAssessment` (`perception/src/models/
collision.py`) -- reuses `Point2D`/`Velocity2D`/`ObjectClassification` (Phase 0/6) and
`VehiclePose` (Phase 8, composed into the new `VehicleState`) rather than duplicating them; the
only genuinely new concepts are the risk classification and the per-object/per-scan result
containers themselves, matching docs/data-model.md "Extensibility rule". `distance` is
**recomputed fresh** from `vehicle_state`, not reused from `DetectedObject.distance` (which
always assumes the vehicle is at the world origin) -- correctness-critical once a future phase
supplies a genuinely moving vehicle pose.

## Explainability

Every result carries a non-empty `reason: list[str]` -- the exact rule-cascade branch that fired,
with the relevant numbers inlined:

```
CRITICAL
  Object is inside the projected vehicle path.
  Estimated TTC = 1.17s (<= critical threshold 2.00s).
```

```
WARNING
  Object is inside the projected vehicle path.
  Distance = 3.50m (<= warning distance 5.00m).
```

## Clear separation of concepts

`distance`, `in_projected_path`, `ttc`, and `risk_level` are four different fields answering four
different questions -- `distance = 3m` does **not** by itself mean imminent collision (an
object 3m directly to the side is `SAFE`; the same distance dead ahead and approaching may be
`CRITICAL`). Full directional (front/rear/left/right) clearance analysis is explicitly out of
scope here -- that is Phase 10's concern.

## Configuration

All in `common.config.Settings`, `LIDAR_`-prefixed env-var overridable, nothing hard-coded. Full
reasoning behind every default is in `common/config.py`'s own comments.

| Setting | Default | Meaning |
|---|---|---|
| `vehicle_length_m` / `vehicle_width_m` | `4.5` / `1.8` | Reused from Phase 0 (reserved "for Phase 9-10"). |
| `front_safety_margin_m` / `rear_safety_margin_m` | `1.0` / `0.5` | Longitudinal buffer, asymmetric. |
| `left_safety_margin_m` / `right_safety_margin_m` | `0.3` / `0.3` | Lateral buffer. |
| `collision_warning_distance_m` / `collision_critical_distance_m` | `5.0` / `2.0` | Proximity-based buffer-zone thresholds. |
| `collision_warning_ttc_s` / `collision_critical_ttc_s` | `4.0` / `2.0` | TTC-based thresholds. |
| `collision_prediction_horizon_s` | `5.0` | How far ahead the discrete simulation looks. |
| `collision_simulation_step_s` | `0.1` | Simulation time step. |
| `collision_minimum_closing_speed_mps` | `0.05` | Noise floor below which TTC is undefined. |
| `collision_minimum_object_radius_m` | `0.15` | Floor on an object's own contact-gap half-extent. |
| `collision_default_vehicle_speed_mps` | `0.0` | Assumed speed when no `VehicleState` is supplied. |

### Parameter selection

TTC thresholds (`2.0s` critical / `4.0s` warning) reflect the same order of magnitude commonly
cited for forward-collision-warning alert timing (perception + reaction + initial braking
response takes roughly `1-2s` for an attentive driver, so a `2.0s` critical alert leaves very
little margin -- intentionally, CRITICAL should mean genuinely imminent). **These are prototype
defaults, not derived from or validated against a specific regulatory standard.** Distance
thresholds were chosen so this phase's own explicit example (a stationary wall `10m` ahead of a
stationary vehicle) reports SAFE, not a false alarm, while an object well inside the vehicle's own
safety envelope (`2.0m`, comfortably inside `front_safety_margin_m + vehicle_length_m/2 = 3.25m`)
is always flagged regardless of current motion.

## Ground-truth evaluation

`scripts/evaluate_collision.py`: for the two moving-obstacle scenarios (`07_moving_crossing`,
`08_approaching_obstacle`), compares the real pipeline's assessment (noisy detection ->
classification -> tracking -> collision) against an idealized one computed by running the exact
same `collision.ttc`/`collision.prediction`/`collision.risk` functions directly against the
simulator's own known, noise-free obstacle position and velocity. This isolates *upstream
perception error* from the collision math itself (already validated independently via hand-
computed unit tests). **Does not measure real-world safety performance** -- only how closely, in
this simulator, the perception-driven assessment tracks an idealized ground-truth one. See
"Ground Truth" in the completion report for the actual numbers from a run.

## Performance

`scripts/benchmark_collision.py` measures `CollisionRiskEngine.evaluate()` alone (synthetic
1/5/10/50-object scans) and the full preprocessing-through-collision chain against real
scenarios. The collision engine itself is sub-millisecond even at 50 objects; the first scan of
any run carries a one-time ~1.5-1.7s "cold start" cost (`sklearn`/`numpy` first-call/import
overhead, not a per-scan algorithmic cost -- confirmed by direct per-scan profiling, isolated to
scan index 0 specifically) that should be excluded when reading `max` timings. See "Performance"
in the completion report for the actual measured numbers.

## Visualization

`collision.visualize.plot_collision_assessment()` (matplotlib, optional `viz` extra): vehicle
body, margin-expanded safety envelope, tracked objects color-coded by risk (green/orange/red),
velocity vectors, and the predicted collision point (red X) if any.

```
python scripts/visualize_collision.py --scenario 08_approaching_obstacle --scans 20 --vehicle-speed 0 --visualize --explain
```

## Self-return guard

`CollisionRiskEngine.evaluate_object` short-circuits any object whose centroid is within
`min_valid_distance_m` (`LIDAR_MIN_VALID_DISTANCE_M`, default `0.30 m`) of the sensor: it returns
`risk_level = SAFE`, `ttc = None`, `in_projected_path = False`, `collision_predicted = False`, and
a `reason` explaining it was treated as an ego-vehicle/sensor self return.

Preprocessing already removes self returns before any cluster can form -- both this radial cutoff
AND the **geometric ego-vehicle footprint mask** (`common.geometry.EgoFootprint`,
`preprocessing.validation` `INSIDE_EGO_FOOTPRINT`), which is what catches a self-return *arc* off
the vehicle body a few tens of centimetres out (the live-rig `#track-1` at ~0.4 m). So a *fresh*
cluster cannot form from a self return. This guard is the residual catch for a track created
*before* the mask took effect and now **coasting** on its stale last-known centroid for a few
scans before the tracker lets it expire -- without the guard, `assess_risk`'s distance rule would
report CRITICAL and `compute_ttc` `0.0 s` off a physically meaningless position (this is the exact
failure the live
ESP32 rig showed: a `track-38`, `vehicle_like`, "~0.2 m", CRITICAL, MIN TTC `0.0`). The object
still appears in `results` (so `object_count` stays honest) and ages out via the normal track
lifecycle; it simply never drives risk or TTC.

## Limitations

- **Prototype, not certified.** See "Status" -- every threshold is a reasoned default, not a
  validated safety parameter.
- **1D longitudinal TTC** is a simplification of true 2D motion; `collision_predicted` (2D
  simulation) is the more complete cross-check, but is itself horizon-bounded and constant-
  velocity.
- **Object footprint approximation** assumes `width`/`.depth` align with the vehicle's own
  heading axes -- exactly true under the default identity pose, not guaranteed under a
  non-identity one (see "Object footprint approximation").
- **No occupancy-grid awareness.** Deliberately, per this phase's spec -- the grid remains
  available for a future phase to add as supporting information.
- **Missing object velocity is assumed stationary**, a documented but not risk-free
  simplification (flagged explicitly in `reason` when it applies).
- **Single scalar vehicle speed** -- no independent lateral/steering velocity is modeled.
- **No occlusion/shadow reasoning** -- an object hidden behind another isn't specially handled
  beyond whatever the underlying tracker already provides.

## Directional clearance (Phase 10)

**Implemented**, in `perception/src/clearance/` (`geometry.py` / `risk.py` / `engine.py`, mirroring
`collision/`'s own three-module split and the same stateless `__init__(settings)` +
`evaluate(...)` shape as `CollisionRiskEngine`). Answers a different question from the collision
engine above: not "is a specific tracked object dangerous", but "how much physical room is there
around the vehicle right now, in each of four directions" -- independent of whether
clustering/classification/tracking has identified anything there yet.

**Input**: the raw post-preprocessing `CartesianScan` (every valid LiDAR return for the scan), not
tracked objects -- a wall or a just-appeared obstacle registers immediately, not only once it has
an established track. `ClearanceEngine.evaluate(cartesian_scan, vehicle_state)` returns a
`ClearanceAssessment` (`models/clearance.py`).

**Directional quadrants**: `clearance.geometry.classify_quadrant` reuses `collision.geometry.
to_vehicle_frame`'s exact vehicle-local `(along, lateral)` frame -- there is exactly one
vehicle-frame transform in this project, not two independently derived ones. Four non-overlapping
90-degree quadrants, boundaries at the 45-degree diagonals: FRONT `[-45, 45)`, LEFT `[45, 135)`,
REAR the remainder behind, RIGHT `[-135, -45)` -- every point belongs to exactly one quadrant.

**Per-direction distance**: for the closest qualifying point in each quadrant, the gap from that
direction's vehicle *safety-margin envelope edge* (`collision.geometry.vehicle_footprint` -- the
same envelope Phase 9 already uses, not a separate one) to the point, floored at `0` (a point
already inside the envelope reports contact, not negative clearance). A qualifying point must be
`valid`, within `lidar_range_max_m`, **and** at least `min_valid_distance_m` from the sensor --
the same self-return cutoff preprocessing applies, re-checked here as defense-in-depth so a
coasting near-origin track (or any stray ~0 m return) cannot force a false `0.00 m` REAR/LEFT/
RIGHT reading via the floor above. A quadrant with no qualifying point (nothing within
`lidar_range_max_m`, or no valid return at all) defaults to `lidar_range_max_m` minus that
direction's envelope offset -- **"no return" is treated as free space out to sensor range, never
as zero clearance** -- the same convention preprocessing/mapping already use elsewhere for a
missing return, not a new rule invented here.

**Aggregates**:
- `min_clearance_m` / `min_direction` -- the smallest of the four directional readings.
- `corridor_width_m` -- `left.distance_m + right.distance_m + (vehicle envelope width)`, i.e. the
  total lateral gap actually available (narrows correctly in `06_narrow_corridor`).

**Clearance classification** (`clearance.risk.assess_clearance`): the same "most-severe-first
cascade + human-readable reasons" approach `collision.risk.assess_risk` established, applied to
`min_clearance_m` against three configured thresholds (`clearance_caution_distance_m` = 1.0m,
`clearance_low_distance_m` = 0.6m, `clearance_critical_distance_m` = 0.3m, in `common/config.py`):

```
min_clearance_m <= critical (0.3m)  -> CRITICAL
min_clearance_m <= low (0.6m)        -> LOW_CLEARANCE
min_clearance_m <= caution (1.0m)     -> CAUTION
otherwise                              -> SAFE
```

This exact four-state name set (`SAFE`/`CAUTION`/`LOW_CLEARANCE`/`CRITICAL`, distinct from
`RiskLevel`'s three) was specified here before the engine existed -- matching a decision already
made, not inventing new terminology.

**Purely geometric, unlike collision**: `vehicle_state.speed_mps` does not affect any clearance
reading -- only `vehicle_state.pose` (position/heading) does. There is no TTC-style closing-speed
reasoning here; clearance is "how much room right now", not "how urgently is it closing".

**Wire protocol**: `serialization.build_clearance_payload` (mirrors `build_risk_payload`'s
None-in/None-out contract exactly) packs a `ClearanceAssessment` into the `PERCEPTION_FRAME`
message's `data.clearance` field -- see docs/communication.md "Perception frame" for the schema
Unity/any other consumer parses. `scripts/serve_unity_bridge.py` runs `ClearanceEngine` every scan
alongside `CollisionRiskEngine`.

**Limitations**: same disclaimer as the collision engine above (prototype, not certified); the
four quadrants are simple 90-degree sectors, not a continuous angular sweep, so a nearest point
right at a 45-degree seam is assigned to one adjacent quadrant, not split between both.
