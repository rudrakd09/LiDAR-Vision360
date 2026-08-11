"""Sensor noise model: Gaussian distance noise, outliers, and missing measurements.

All randomness flows through a single seeded `random.Random` instance so an entire simulated
session (scan after scan) is fully reproducible given the same seed -- required for deterministic
tests (see `tests/test_noise.py`, `tests/test_scenarios.py`).
"""

from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass
class NoiseConfig:
    distance_noise_std_m: float = 0.0
    outlier_probability: float = 0.0
    missing_probability: float = 0.0
    seed: int | None = None


class NoiseModel:
    """Applies `NoiseConfig` to a single ground-truth distance measurement."""

    def __init__(self, config: NoiseConfig) -> None:
        self.config = config
        self._rng = random.Random(config.seed)

    def apply(self, true_distance: float | None, range_min_m: float, range_max_m: float) -> tuple[float, bool]:
        """Return `(measured_distance, valid)` for one ray.

        `true_distance` is `None` when nothing was hit within range (reported as `range_max_m`,
        `valid=True` -- a real LiDAR typically reports max range rather than no data at all for
        free space). A "missing measurement" (`valid=False`) is a distinct, explicitly modeled
        failure mode (dropped return), not the same thing as "nothing in range".
        """
        if self._rng.random() < self.config.missing_probability:
            return 0.0, False

        distance = range_max_m if true_distance is None else max(range_min_m, min(true_distance, range_max_m))

        if self.config.distance_noise_std_m > 0.0:
            distance += self._rng.gauss(0.0, self.config.distance_noise_std_m)

        if self._rng.random() < self.config.outlier_probability:
            distance = self._rng.uniform(range_min_m, range_max_m)

        distance = max(0.0, min(distance, range_max_m))
        return distance, True
