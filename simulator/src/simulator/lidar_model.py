"""LiDAR sampling model: turns a configured angular resolution into the list of angles sampled
each sweep. Arbitrary resolution is supported -- 360 points is simply the default (1 degree)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LiDARModel:
    range_min_m: float
    range_max_m: float
    angular_resolution_deg: float = 1.0
    scan_frequency_hz: float = 10.0

    @property
    def num_points(self) -> int:
        """Number of angular samples per 360° sweep, derived from `angular_resolution_deg`."""
        return max(1, round(360.0 / self.angular_resolution_deg))

    def angles_deg(self) -> list[float]:
        """The evenly-spaced angles (degrees, `[0, 360)`) sampled in one sweep."""
        n = self.num_points
        return [(i * 360.0 / n) % 360.0 for i in range(n)]
