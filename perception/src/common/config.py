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

    # --- Vehicle / safety parameters (used from Phase 9-10 onward) ---
    vehicle_width_m: float = 1.8
    vehicle_length_m: float = 4.5

    # --- Cloud backend (used from Phase 12 onward) ---
    backend_host: str = "0.0.0.0"
    backend_port: int = 8000

    # --- Database (used from Phase 13 onward) ---
    database_url: str = "postgresql://lidar:lidar@localhost:5432/lidar_vision360"


@lru_cache
def get_settings() -> Settings:
    """Return the process-wide Settings singleton (cached; re-read by restarting the process)."""
    return Settings()
