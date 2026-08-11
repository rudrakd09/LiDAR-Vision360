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

    # --- Cloud backend (used from Phase 12 onward) ---
    backend_host: str = "0.0.0.0"
    backend_port: int = 8000

    # --- Database (used from Phase 13 onward) ---
    database_url: str = "postgresql://lidar:lidar@localhost:5432/lidar_vision360"


@lru_cache
def get_settings() -> Settings:
    """Return the process-wide Settings singleton (cached; re-read by restarting the process)."""
    return Settings()
