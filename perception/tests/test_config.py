"""Tests for common.config.Settings / get_settings."""

from common.config import Settings, get_settings


class TestSettings:
    def test_defaults(self):
        settings = Settings(_env_file=None)
        assert settings.lidar_num_points == 360
        assert settings.lidar_range_min_m == 0.05
        assert settings.lidar_range_max_m == 12.0
        assert settings.backend_port == 8000

    def test_env_prefix_override(self, monkeypatch):
        monkeypatch.setenv("LIDAR_LIDAR_NUM_POINTS", "720")
        monkeypatch.setenv("LIDAR_LOG_LEVEL", "DEBUG")
        settings = Settings(_env_file=None)
        assert settings.lidar_num_points == 720
        assert settings.log_level == "DEBUG"

    def test_get_settings_is_cached(self):
        assert get_settings() is get_settings()
