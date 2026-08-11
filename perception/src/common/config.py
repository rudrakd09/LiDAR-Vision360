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

    # --- Cloud backend (used from Phase 12 onward) ---
    backend_host: str = "0.0.0.0"
    backend_port: int = 8000

    # --- Database (used from Phase 13 onward) ---
    database_url: str = "postgresql://lidar:lidar@localhost:5432/lidar_vision360"


@lru_cache
def get_settings() -> Settings:
    """Return the process-wide Settings singleton (cached; re-read by restarting the process)."""
    return Settings()
