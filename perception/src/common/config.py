"""Centralized, environment-driven configuration.

Nothing in this project should hard-code file paths, vehicle dimensions, LiDAR range, safety
thresholds, network addresses, ports, or database credentials (see PROJECT_SPECIFICATION.md,
Quality Requirements). Instead, every such value lives on `Settings` below, with a sane default
and an override via environment variable / `.env` file, prefixed `LIDAR_`.

Usage:
    from common.config import get_settings
    settings = get_settings()
    print(settings.lidar_num_points)
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

# perception/src/common/config.py -> parents[3] == repository root
_REPO_ROOT = Path(__file__).resolve().parents[3]


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_prefix="LIDAR_",
        env_file=str(_REPO_ROOT / ".env"),
        env_file_encoding="utf-8",
        extra="ignore",
    )

    # --- General ---
    environment: str = "development"
    log_level: str = "INFO"

    # --- LiDAR / simulator parameters (used from Phase 2 onward) ---
    lidar_range_min_m: float = 0.05
    lidar_range_max_m: float = 12.0
    lidar_num_points: int = 360
    lidar_scan_frequency_hz: float = 10.0

    # --- LiDAR angular sampling and noise (Phase 2, simulator/) ---
    # `lidar_num_points` above remains the knob used by the minimal foundation
    # placeholder (perception.datasources.simulated). The full simulator instead derives its
    # point count from angular resolution, so resolutions like 0.5deg are expressible directly.
    lidar_angular_resolution_deg: float = 1.0
    lidar_distance_noise_std_m: float = 0.02
    lidar_outlier_probability: float = 0.0
    lidar_missing_probability: float = 0.0
    lidar_random_seed: int | None = None

    # --- Vehicle / safety parameters (used from Phase 9-10 onward) ---
    vehicle_width_m: float = 1.8
    vehicle_length_m: float = 4.5

    # Safety-margin buffer added around the bare vehicle body on each side, forming the
    # "warning zone" envelope collision.CollisionRiskEngine (Phase 9) and the future clearance
    # engine (Phase 10) both reason about -- kept here alongside vehicle_width_m/vehicle_length_m
    # rather than siloed under a `collision_` prefix, since both phases need the same envelope.
    # front > rear: more buffer is warranted ahead (the vehicle's primary direction of travel and
    # stopping-distance exposure) than behind; left/right are modest lateral clearance (e.g.
    # mirror/door swing), symmetric by default since nothing in this project's model yet
    # distinguishes a driver/passenger side.
    front_safety_margin_m: float = 1.0
    rear_safety_margin_m: float = 0.5
    left_safety_margin_m: float = 0.3
    right_safety_margin_m: float = 0.3

    # --- Vehicle-mounted LiDAR pose (Phase 2). Defaults to vehicle center, no rotation. ---
    lidar_mount_x_m: float = 0.0
    lidar_mount_y_m: float = 0.0
    lidar_mount_orientation_deg: float = 0.0

    # --- Preprocessing (Phase 3, perception/src/preprocessing/) ---
    # Range validation reuses lidar_range_min_m / lidar_range_max_m above rather than
    # duplicating them -- there is exactly one configured sensor range in the system.
    #
    # Local outlier detector (Hampel-style): a measurement is flagged as an outlier if it
    # deviates from the median of its `preprocessing_outlier_window_size` angular neighbors
    # (circular, wraps at 0/360) by more than `preprocessing_outlier_threshold_m`.
    preprocessing_outlier_threshold_m: float = 0.5
    preprocessing_outlier_window_size: int = 5

    # Spatial median filter (per-scan, angle-ordered, circular): smooths minor jitter while
    # remaining edge-preserving. `1` disables it (no-op).
    preprocessing_median_filter_window: int = 5

    # Optional temporal (cross-scan) exponential smoothing, matched per angle bin:
    # filtered_t = alpha * current_t + (1 - alpha) * previous_filtered_t.
    # Disabled by default -- it trades responsiveness (lag on moving obstacles) for smoothness,
    # so it is opt-in rather than applied unconditionally.
    preprocessing_temporal_filter_enabled: bool = False
    preprocessing_temporal_filter_alpha: float = 0.5

    # --- Clustering (Phase 5, perception/src/clustering/) ---
    # DBSCAN on (x, y): a point is a core point if >= clustering_min_samples points (including
    # itself) lie within clustering_eps_m of it; core points within eps of each other join the
    # same cluster.
    #
    # These defaults were reached empirically, not just by theory -- see docs/clustering.md
    # "Parameter selection" for the full walkthrough. The naive theoretical starting point
    # (eps=0.3m, min_samples=4, sized to just bridge the ~0.21m worst-case adjacent-point arc
    # spacing at 12m/1deg resolution) turned out wrong in practice: LiDAR surface points form a
    # quasi-1D arc, so a point's eps-neighborhood mostly only contains its immediate line
    # neighbors, not a full 2D neighborhood's worth -- at eps=0.3m a wall point typically has
    # only 2 neighbors within range, one short of the 3 *other* points min_samples=4 requires,
    # so entire real walls were incorrectly classified as 100% noise. eps=0.6m / min_samples=3
    # (validated against all 10 scenarios, including explicit too-small/too-large sweeps on both
    # parameters) reliably detects a single wall/pole/vehicle as one cluster, keeps the narrow
    # corridor's two walls (2m apart) and every other genuinely-distinct simulator obstacle
    # separate, and only starts merging the corridor's walls once eps approaches that 2m gap
    # directly (observed at eps=2.0m, over 3x this default).
    clustering_eps_m: float = 0.6
    clustering_min_samples: int = 3

    # Independent post-filter: a DBSCAN cluster smaller than this is demoted to noise. Every
    # DBSCAN cluster already has >= clustering_min_samples points by construction, so this is a
    # no-op at its default (equal to min_samples) -- it only has an effect when raised above
    # min_samples, for callers who want extra noise robustness without changing DBSCAN's own
    # density parameter.
    clustering_min_cluster_points: int = 3

    # Points within this margin of lidar_range_max_m are treated as "no return"/free space, not
    # obstacle-candidate points, and are excluded from clustering entirely (counted as noise
    # instead) -- see docs/clustering.md "Free-space filtering" for why this is necessary.
    clustering_max_range_margin_m: float = 0.2

    # --- Classification (Phase 6, perception/src/objects/) ---
    # Geometry-based rule scoring, see docs/object-classification.md "Classification method" and
    # "Parameter selection" for the full reasoning behind every value below.
    #
    # The one threshold that decides "confident category" vs. UNKNOWN: the winning category's
    # score must clear this to be reported; otherwise the object is UNKNOWN (with that best
    # score still recorded as its confidence, for transparency about how close it came).
    classification_min_confidence: float = 0.55

    # WALL: visible extent shorter than this isn't confidently a wall (could be a short fragment
    # or fence-post edge) even if highly linear.
    classification_wall_min_length_m: float = 1.0

    # WALL: perpendicular thickness (the cluster's *other*, smaller extent) above this isn't
    # flat enough to be a wall -- rules out boxy/2-faced clusters (e.g. a vehicle seen at an
    # angle) that can otherwise still show high linearity along their dominant axis.
    classification_wall_max_thickness_m: float = 0.6

    # POLE_LIKE: extent larger than this is too big to be a thin pole/post, even if the fit is
    # circular (a curved vehicle panel can locally look circular too -- size gates it out).
    classification_pole_max_extent_m: float = 0.8

    # VEHICLE_LIKE: approximate size envelope, deliberately a *range* (not the ego vehicle's own
    # exact dimensions in vehicle_width_m/vehicle_length_m) covering small-car to small-van/truck
    # silhouettes as seen from one side by a 2D LiDAR.
    classification_vehicle_width_min_m: float = 1.0
    classification_vehicle_width_max_m: float = 2.6
    classification_vehicle_depth_min_m: float = 1.5
    classification_vehicle_depth_max_m: float = 6.0
    classification_vehicle_min_points: int = 6

    # PERSON_LIKE: intentionally narrow and conservative -- see docs/object-classification.md
    # "Person-like classification" for the full caveats. A single 2D LiDAR scan cannot reliably
    # identify a human; this category exists but is capped well below full confidence.
    classification_person_extent_min_m: float = 0.15
    classification_person_extent_max_m: float = 0.9
    classification_person_max_confidence: float = 0.6

    # LARGE_OBSTACLE: the generic catch-all for "clearly substantial, but not a confident match
    # for any specific category" -- slightly larger than the wall-length minimum, since this
    # category is about overall bulk, not linear extent.
    classification_large_min_extent_m: float = 1.2

    # --- Tracking (Phase 7, perception/src/tracking/) ---
    # Object association: nearest-neighbour, gated by Euclidean distance between a detection's
    # centroid and a track's Kalman-predicted position. The scenarios this phase must handle
    # (07_moving_crossing: 1.5 m/s, 08_approaching_obstacle: 2.0 m/s, both at the default 10Hz
    # scan rate -> <=0.2m true displacement per scan) all sit comfortably inside this gate even
    # allowing for prediction error and a missed scan or two, while still being far tighter than
    # the ~0.6m clustering_eps_m gap between genuinely distinct simulator obstacles -- see
    # docs/tracking.md "Association algorithm" for the full reasoning and sensitivity notes.
    tracking_max_association_distance_m: float = 2.0

    # Secondary, *soft* association cost terms -- neither is a hard gate (only distance is), so a
    # track is never refused a plainly-correct match just because its cluster's size estimate was
    # noisy or its classification flipped near a threshold. `tracking_dimension_cost_weight`
    # scales the (width-diff + depth-diff) term before adding it to the distance cost;
    # `tracking_max_dimension_diff_m` caps how much that term alone can ever contribute, so one
    # wildly-misestimated cluster can't dominate the distance-based decision.
    tracking_dimension_cost_weight: float = 0.3
    tracking_max_dimension_diff_m: float = 1.5

    # Flat cost penalty added when a track's and a candidate detection's classifications differ
    # (and neither is UNKNOWN) -- again soft, not a hard exclusion, because classification can
    # legitimately flip frame-to-frame near objects.classification_min_confidence (see
    # docs/object-classification.md); per this phase's spec, classification must never be
    # *mandatory* for association.
    tracking_classification_mismatch_penalty: float = 0.5

    # Track lifecycle: a new track must be associated this many times (including its creating
    # detection) before it graduates TENTATIVE -> CONFIRMED -- filters out single-scan spurious
    # clusters (sensor noise, a transient clustering artifact) from ever being reported as a
    # confident, persistent object.
    tracking_min_hits_to_confirm: int = 3

    # How long an unmatched track is kept alive (as COASTING, reporting a predicted-only
    # position) before being marked LOST -- two independent knobs, either one crossing its
    # threshold is sufficient: a scan-count budget (robust to variable scan rate) and a wall-clock
    # backstop (catches a stalled/paused data source that would otherwise never "use up" its scan
    # budget). Handles brief LiDAR occlusion/missed detections without discarding the track's
    # identity and velocity history.
    tracking_max_missed_scans: int = 5
    tracking_track_timeout_s: float = 1.0

    # Kalman filter (constant-velocity model, state [x, y, vx, vy]) -- see docs/tracking.md
    # "Kalman filter" for the full state/process/measurement model. `process_noise_std` is the
    # assumed standard deviation of unmodeled acceleration (m/s^2), driving how much the filter
    # trusts its own constant-velocity prediction vs. new measurements; `measurement_noise_std_m`
    # is the assumed centroid position measurement noise (m), driving how much it trusts each new
    # detection. Both were set from this project's own numbers rather than picked arbitrarily:
    # measurement noise is dominated by `lidar_distance_noise_std_m` (0.02m) averaged down across
    # a cluster's points *and* by cluster-boundary/DBSCAN-membership jitter shifting the centroid
    # itself scan-to-scan, which empirically dominates -- 0.15m reflects that observed centroid
    # jitter, not the raw per-point sensor noise. Process noise (0.5 m/s^2) is a generic
    # "moderate, unpredictable acceleration" assumption (a pole/vehicle/pedestrian in these
    # scenarios moves at constant velocity by construction, but the filter must not be so rigid
    # it can't track a real turn or a speed change) -- see docs/tracking.md "Parameter selection"
    # for the tuning walkthrough against the two moving-obstacle scenarios.
    tracking_process_noise_std_mps2: float = 0.5
    tracking_measurement_noise_std_m: float = 0.15
    tracking_initial_position_uncertainty_m: float = 1.0
    tracking_initial_velocity_uncertainty_mps: float = 5.0

    # Velocity is not reported (stays `None` on DetectedObject.velocity) until a track has this
    # many real (non-coasting) measurement updates -- a 1-2 point velocity estimate off a
    # just-created track is dominated by measurement noise, not real motion; see docs/tracking.md
    # "Velocity estimation".
    tracking_min_observations_for_velocity: int = 3

    # Movement classification: STATIONARY below this speed, MOVING at or above it. Set above the
    # residual velocity noise a perfectly stationary object's Kalman-filtered estimate still shows
    # (driven by `tracking_measurement_noise_std_m` and the scan-to-scan dt) and well below both
    # moving-obstacle scenarios' actual speeds (1.5 m/s, 2.0 m/s) -- see docs/tracking.md
    # "Movement classification" for the reasoning and the noise-floor measurement it's based on.
    tracking_stationary_speed_threshold_mps: float = 0.3

    # --- Mapping (Phase 8, perception/src/mapping/) ---
    # A persistent 2D occupancy grid (FREE/OCCUPIED/UNKNOWN) built from successive CartesianScans
    # -- see docs/mapping.md for the full write-up. This is a 2D LiDAR occupancy map, not a true
    # 3D map (see docs/architecture.md "Sensor limitation").
    #
    # Grid extent/resolution. 40m x 40m at 0.1m/cell -> 400x400 = 160,000 cells (a few hundred KB
    # as int8 + float64 arrays -- trivial memory), comfortably covering the sensor's full
    # lidar_range_max_m (12.0m) radius around the vehicle in every direction with headroom to
    # spare, and fine enough to resolve the simulator's smallest obstacle (03_pole_left's 0.15m
    # radius pole spans ~3 cells) without the per-scan update cost (proportional to grid area
    # touched, not total grid size, since only cells along actual rays are ever written) becoming
    # a real-time concern -- see docs/mapping.md "Performance".
    mapping_width_m: float = 40.0
    mapping_height_m: float = 40.0
    mapping_resolution_m: float = 0.1

    # How far a ray is trusted to mark FREE space -- independently configurable from
    # lidar_range_max_m (unlike e.g. preprocessing's reuse of lidar_range_min_m/_max_m for range
    # *validation*, "there is exactly one configured sensor range in the system"): an operator may
    # want the *map* to trust a shorter, more reliable distance than the sensor's raw physical
    # max range even without changing the sensor's own configured range -- same reasoning
    # clustering_max_range_margin_m already established as its own knob layered on top of
    # lidar_range_max_m rather than a strict reuse. Defaults equal to lidar_range_max_m's own
    # default (12.0) since there is no reason to differ out of the box.
    mapping_max_range_m: float = 12.0
    # A measurement within this margin of mapping_max_range_m is treated as "no return"/free
    # space up to mapping_max_range_m, not a real obstacle hit -- same "how close to max range
    # counts as no signal" concept clustering_max_range_margin_m already established (see
    # docs/clustering.md "Free-space filtering"), given its own knob here rather than reused
    # directly since this phase's mapper and Phase 5's clusterer are independent consumers that
    # may reasonably want to tune it separately.
    mapping_no_return_margin_m: float = 0.2

    # Log-odds occupancy update magnitudes -- see docs/mapping.md "Occupancy update model" for
    # the full probabilistic derivation. occupied_update (l = +0.85) corresponds to an inverse
    # sensor model of P(occupied | hit) ~= 0.70; free_update (l = -0.4, smaller magnitude)
    # corresponds to P(occupied | ray passes through) ~= 0.40 -- deliberately weaker than the
    # occupied update (occupied evidence is intentionally "stickier" than free evidence: a single
    # false pass-through should not erase a real obstacle's evidence as fast as a single real hit
    # establishes it, which favors safety).
    mapping_free_update: float = 0.4
    mapping_occupied_update: float = 0.85

    # Log-odds are clamped to this range after every update, preventing unbounded growth (a cell
    # observed occupied/free thousands of times must not become "infinitely" confident, both for
    # numerical sanity and so a handful of contradicting observations can still move it). max
    # (+3.5, p ~= 0.97) and min (-2.0, p ~= 0.12) were chosen so a stationary obstacle/free cell
    # reinforced every scan converges to a confident, stable plateau within a handful of scans
    # without being literally unbounded.
    mapping_min_log_odds: float = -2.0
    mapping_max_log_odds: float = 3.5

    # Discretization thresholds: FREE if log_odds <= free_threshold, OCCUPIED if log_odds >=
    # occupied_threshold, else UNKNOWN. Each is set with smaller magnitude than its corresponding
    # single-observation update (|free_threshold|=0.3 < |free_update|=0.4;
    # |occupied_threshold|=0.7 < |occupied_update|=0.85) so a *single* real observation already
    # crosses into FREE/OCCUPIED territory -- the discretized state reflects current best belief
    # immediately, while the underlying log-odds value keeps accumulating confidence (and
    # resisting a later contradicting observation) with repeated observations. log_odds == 0.0
    # (a cell's initial, never-observed value) correctly falls in neither band, i.e. UNKNOWN.
    mapping_free_threshold: float = -0.3
    mapping_occupied_threshold: float = 0.7

    # Optional decay: pulls every cell's log-odds toward 0 (UNKNOWN) by this fraction at the start
    # of each update() call, before that scan's own ray updates are applied -- models the fact
    # that a previously-occupied cell should not remain permanently occupied forever if the
    # environment changes (see docs/mapping.md "Decay / dynamic environment" for why this mostly
    # matters for cells that stop being reinforced, not for a genuinely-stationary obstacle, which
    # gets fresh reinforcing evidence -- and so counteracts decay -- every single 360 sweep).
    # Defaults off (like preprocessing_temporal_filter_enabled) -- an explicit opt-in, since an
    # overly aggressive setting is exactly what this phase's spec warns against ("do not make the
    # map decay so aggressively that stationary obstacles disappear").
    mapping_decay_enabled: bool = False
    mapping_decay_rate: float = 0.02

    # --- Collision / Risk Engine (Phase 9, perception/src/collision/) ---
    # SAFE/WARNING/CRITICAL risk assessment from tracked-object position/velocity/geometry,
    # vehicle geometry (vehicle_width_m/vehicle_length_m/*_safety_margin_m above), and vehicle
    # state -- see docs/collision.md. This is a prototype collision-awareness system, not a
    # certified automotive safety system.
    #
    # Distance thresholds: an in-path object this close is flagged regardless of current relative
    # motion (it may start moving, or the vehicle may) -- distinct from the TTC thresholds below,
    # which react to closing motion specifically. critical_distance (2.0m) is near-contact range
    # (comfortably inside front_safety_margin_m + vehicle_length_m/2, i.e. already within the
    # vehicle's own safety envelope); warning_distance (5.0m) is a generic "getting close" buffer.
    # Chosen so a stationary, in-path object well outside these (e.g. a wall 10m ahead of a
    # stationary vehicle, this phase's own explicit example) reports SAFE, not a false alarm.
    collision_warning_distance_m: float = 5.0
    collision_critical_distance_m: float = 2.0

    # TTC thresholds, seconds. 2.0s (critical) / 4.0s (warning) reflect the same order of
    # magnitude commonly cited for forward-collision-warning alert timing (roughly: perception +
    # reaction + initial braking response takes on the order of 1-2s for an attentive driver, so a
    # critical alert at 2.0s TTC leaves very little margin -- intentionally, since CRITICAL should
    # mean genuinely imminent -- while 4.0s gives a warning with real time to react). Not derived
    # from a specific regulatory standard -- see docs/collision.md "Parameter selection" for the
    # full caveat that these are prototype defaults, not validated safety-certified thresholds.
    collision_warning_ttc_s: float = 4.0
    collision_critical_ttc_s: float = 2.0

    # How far into the future collision_predicted/predicted_collision_time/_position (the
    # discrete footprint-intersection simulation) looks, and the simulation's time step. 5s at
    # 0.1s steps (50 steps) is cheap to compute per object per scan and comfortably covers both
    # configured TTC thresholds above with margin, without extrapolating constant-velocity motion
    # so far into the future that "do not claim centimeter-level accuracy" would be violated.
    collision_prediction_horizon_s: float = 5.0
    collision_simulation_step_s: float = 0.1

    # Below this closing speed (m/s), relative motion along the vehicle's heading axis is treated
    # as "not meaningfully approaching" -- ttc becomes None (undefined/infinite) rather than an
    # enormous or noise-driven finite number. Small relative to any real approach speed of
    # interest (this project's moving-obstacle scenarios use 1.5-2.0 m/s) but large enough to
    # absorb residual Kalman-filtered velocity-estimate noise for a nominally-stationary object.
    collision_minimum_closing_speed_mps: float = 0.05

    # Floor on an object's own half-extent (meters) when computing TTC/collision-prediction
    # contact gaps -- defends against a degenerate/tiny detection (e.g. a 2-3 point cluster)
    # producing an unrealistically small effective footprint. Matches the simulator's own
    # smallest modeled obstacle (03_pole_left's 0.15m-radius pole).
    collision_minimum_object_radius_m: float = 0.15

    # Vehicle forward speed (m/s) `CollisionRiskEngine.evaluate()` assumes when the caller doesn't
    # supply a `VehicleState` -- 0.0 (stationary) is the only assumption that never *overstates*
    # risk when the real value is unknown. See docs/collision.md "Vehicle velocity assumption":
    # the API always accepts a real `VehicleState` (with `speed_mps` supplied from an actual
    # speedometer/localization source in a future phase); this default only applies when none is
    # given at all.
    collision_default_vehicle_speed_mps: float = 0.0

    # Hysteresis: how far beyond the plain thresholds above real distance/TTC must recover before
    # a risk level is allowed to DE-escalate (CRITICAL->WARNING->SAFE). Escalating (getting worse)
    # is always immediate/unfiltered -- only recovery is delayed, so genuine danger is never
    # masked; see collision.hysteresis for the mechanism (a small per-track_id "last accepted
    # level" memory in CollisionRiskEngine -- assess_risk's own threshold rules are unchanged).
    # Defaults (0.5m / 0.5s) comfortably absorb the residual sensor/preprocessing noise already
    # documented above (lidar_distance_noise_std_m default 0.02m) -- found via a real repro: a
    # wall placed exactly at collision_warning_distance_m (5.0m) flip-flopped SAFE<->WARNING on
    # nearly every single scan from sub-millimeter jitter, with no code bug anywhere in the
    # bridge/backend/dashboard pipeline -- the risk engine itself had no deadband at its boundary.
    collision_risk_hysteresis_distance_margin_m: float = 0.5
    collision_risk_hysteresis_ttc_margin_s: float = 0.5

    # --- Clearance Engine (Phase 10, perception/src/clearance/) ---
    # Directional (front/rear/left/right) clearance thresholds, meters -- from the vehicle's
    # safety-margin envelope edge (collision.geometry.vehicle_footprint, the same envelope Phase 9
    # already uses) to the nearest LiDAR return in that direction. Three thresholds (not two, like
    # collision's warning/critical) because docs/collision.md's own Phase 10 stub specified a
    # four-state SAFE/CAUTION/LOW_CLEARANCE/CRITICAL set before this engine existed -- matching a
    # decision already made, not inventing new terminology. Values sit below front_safety_margin_m
    # (1.0) since clearance is about the *remaining* gap beyond that margin, not the margin itself:
    # caution (1.0m) is a generic "getting tight" buffer at roughly the same scale as the existing
    # front margin; low (0.6m) and critical (0.3m) step down from there, critical being near-
    # contact range (comparable in spirit to collision_critical_distance_m's 2.0m, but clearance is
    # measured from the envelope edge rather than the object centroid, so a much smaller absolute
    # value is appropriate here).
    clearance_caution_distance_m: float = 1.0
    clearance_low_distance_m: float = 0.6
    clearance_critical_distance_m: float = 0.3

    # --- Real-time streaming (Phase 12, perception/src/streaming/) ---
    # The Python <-> Unity communication layer: a structured, versioned JSON protocol (see
    # docs/communication.md) alongside the legacy raw <START>/<END> line protocol (unchanged,
    # `streaming_raw_port`). `protocol_version` itself is deliberately NOT a Settings field here
    # (see streaming/protocol.py's own PROTOCOL_VERSION constant) -- it describes what the
    # running code actually implements, not a deployment choice; making it independently
    # configurable would let a misconfigured value lie about the wire format actually in use.
    streaming_host: str = "127.0.0.1"
    streaming_json_port: int = 5006
    streaming_raw_port: int = 5005  # matches the existing LidarTCPClient.cs's hard-coded default -- do not change without updating that script's own default too

    # Liveness: the server sends a HEARTBEAT message every `streaming_heartbeat_interval_s`
    # even when no perception frame was published in that window (e.g. between scans); a client
    # that has received nothing (frame or heartbeat) for `streaming_connection_timeout_s`
    # considers the connection STALE, distinct from a clean TCP disconnect (see
    # docs/communication.md "Heartbeat / connection status"). timeout is deliberately several
    # heartbeat intervals, not a tight 1x multiple, to absorb ordinary local scheduling jitter
    # without flapping.
    streaming_heartbeat_interval_s: float = 2.0
    streaming_connection_timeout_s: float = 6.0
    # Unity-side reconnect attempt spacing after a detected disconnect/timeout -- listed here
    # too (not just as a PerceptionTCPClient Inspector field) so it's documented alongside every
    # other streaming parameter, even though Python's own server never reads it itself.
    streaming_reconnect_interval_s: float = 2.0

    # Per-client outgoing queue depth. A slow/stalled Unity client must never block the
    # perception pipeline (see docs/communication.md "Non-blocking design") -- each client gets
    # its own bounded queue and its own sender thread; once full, the OLDEST queued message is
    # dropped to make room for the newest, never the other way around (the latest state is what a
    # real-time visualization actually needs -- see "Frame dropping"). `2` (not `1`) leaves room
    # for a HEARTBEAT to queue up alongside one still-pending PERCEPTION_FRAME without evicting
    # either immediately, while still bounding memory to a handful of messages, never unbounded.
    streaming_max_outgoing_queue: int = 2

    # Raw-point transmission mode inside a PERCEPTION_FRAME's optional `points` field (off by
    # default at the call site, see scripts/serve_unity_bridge.py --include-points): "polar"
    # (angle/distance, matches the sensor's own native representation and the legacy raw
    # protocol), "cartesian" (x/y, matches what most Unity rendering code actually wants,
    # skipping a client-side trig step), or "both". Configurable because different consumers
    # want different things and there is no single right answer independent of the client.
    streaming_point_mode: str = "polar"

    # Occupancy-map transmission: reuses the exact periodic-downsampled strategy
    # `scripts/serve_unity_bridge.py` already established in Phase 11 (see docs/mapping.md and
    # docs/communication.md "Occupancy map streaming" for why full-resolution-every-scan was
    # rejected) -- centralized here as configuration rather than only CLI flags.
    streaming_map_every_n_scans: int = 5
    streaming_map_downsample: int = 4

    # Sanity ceiling on a single outgoing message's serialized size (bytes) -- see
    # docs/communication.md "Security / validation": "reject absurdly large frames." A
    # legitimate frame (even a full-resolution, undownsampled map) should never approach this;
    # exceeding it indicates a bug (e.g. downsampling disabled on an oversized map) or malicious
    # input, and the message is dropped with a logged warning rather than sent.
    streaming_max_message_bytes: int = 4_000_000

    # --- Cloud backend (local-only; PROJECT_SPECIFICATION.md originally numbered this "Phase 12,"
    # but that number was reassigned to real-time Python<->Unity streaming during actual execution
    # -- see docs/architecture.md "Status" for the as-built phase history). `cloud/backend` --
    # see docs/cloud.md. ---
    backend_host: str = "0.0.0.0"
    backend_port: int = 8000

    # Origins the dashboard dev server is allowed to call the API/WebSocket from (CORS). The Vite
    # default dev port -- kept as a list so a deployed dashboard's real origin can be added via
    # env without code changes.
    backend_cors_origins: list[str] = Field(default_factory=lambda: ["http://localhost:5173", "http://127.0.0.1:5173"])

    # How many of the most recent PERCEPTION_FRAMEs the backend keeps in memory for
    # /api/latest, /api/objects, /api/tracks, and a newly-connected WebSocket client's initial
    # snapshot -- see docs/cloud.md "Do not expose unlimited historical records." Raw frames are
    # never written to the database (unbounded growth); only this bounded ring buffer holds them.
    backend_ring_buffer_size: int = 200

    # Max rows returned by /api/events, /api/collision-events, /api/clearance-events, /api/sessions
    # when the caller doesn't specify a smaller `limit` -- same "do not expose unlimited historical
    # records" reasoning, applied to the database-backed endpoints instead of the ring buffer.
    backend_event_default_limit: int = 50
    backend_event_max_limit: int = 500

    # How long (seconds) a track_id absent from the ring buffer is still reported by /api/tracks
    # (as "recently seen, not currently visible") before being dropped from that roster --
    # mirrors TrackedObjectVisualizer.cs's own removeAfterSeconds grace period on the Unity side,
    # applied here to the same underlying question ("is this track still relevant right now").
    backend_track_grace_period_s: float = 2.0

    # How many of the most recent per-scan position/velocity samples GET /api/tracking-history
    # keeps for each track_id -- bounded like every other in-memory collection here (see
    # backend_ring_buffer_size's own comment), so a long-lived track cannot grow this without
    # bound either. Cleared (along with the rest of that track_id's bookkeeping) once the track
    # itself is pruned from the roster (backend_track_grace_period_s elapsed with no sighting).
    backend_track_history_length: int = 50

    # A session is reported "active" only while a message (frame or heartbeat) has arrived within
    # this many seconds -- distinct from the raw TCP `connection.state` (which stays "connected"
    # even if the bridge process has stalled without actually dropping the socket). See
    # docs/cloud.md "Session lifecycle". Kept as its own setting rather than reusing
    # `streaming_connection_timeout_s` (Unity's own staleness threshold) since the backend and
    # Unity are independent consumers that may reasonably tune this separately.
    backend_session_stale_threshold_s: float = 3.0

    # --- Database (local-only; see the cloud-backend note above -- phase numbers below this point
    # in PROJECT_SPECIFICATION.md's original plan no longer match as-built history). Tried first;
    # if unreachable (no local Postgres, wrong credentials, driver missing), the backend falls back
    # to a local SQLite file at backend_sqlite_fallback_path and logs a warning rather than
    # failing to start -- see docs/cloud.md "Database". ---
    database_url: str = "postgresql://lidar:lidar@localhost:5432/lidar_vision360"
    backend_sqlite_fallback_path: str = str(_REPO_ROOT / "cloud" / "backend" / "data" / "lidar_vision360.db")


@lru_cache
def get_settings() -> Settings:
    """Return the process-wide Settings singleton (cached; re-read by restarting the process)."""
    return Settings()
