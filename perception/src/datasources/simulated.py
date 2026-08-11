"""Minimal placeholder simulated LiDAR data source.

This is intentionally bare-bones: it emits `lidar_num_points` measurements evenly spaced around
360°, at a random-but-plausible distance with light Gaussian noise. It exists so that the
foundation (Phase 0) has an end-to-end, runnable `LiDARDataSource` to exercise the data models
and prove the abstraction works.

The *real* simulator - configurable walls, poles, rectangular obstacles, vehicle-like objects,
moving obstacles, narrow corridors, outliers, dropouts, etc. - is built in `simulator/` as
Phase 2, and will produce `ScanFrame`s through this same `LiDARDataSource` interface (likely via
a thin adapter that wraps the scenario engine), so the perception pipeline built against this
placeholder keeps working unchanged once Phase 2 lands.
"""

from __future__ import annotations

import random
import time
import uuid

from common.config import Settings, get_settings
from datasources.base import LiDARDataSource
from models.lidar import LiDARPoint
from models.scan import ScanFrame


class SimulatedLiDARDataSource(LiDARDataSource):
    """Generates synthetic 360° scans in-process, with no external dependencies."""

    def __init__(self, settings: Settings | None = None, seed: int | None = None) -> None:
        self._settings = settings or get_settings()
        self._rng = random.Random(seed)
        self._connected = False
        self._sequence_number = 0
        self.source_id = "simulated"

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def read_scan(self) -> ScanFrame:
        if not self._connected:
            raise RuntimeError("SimulatedLiDARDataSource.read_scan() called before connect().")

        settings = self._settings
        now = time.time()
        num_points = settings.lidar_num_points
        angle_step = 360.0 / num_points

        points: list[LiDARPoint] = []
        for i in range(num_points):
            angle = (i * angle_step) % 360.0
            base_distance = self._rng.uniform(settings.lidar_range_min_m + 0.5, settings.lidar_range_max_m)
            noisy_distance = max(0.0, base_distance + self._rng.gauss(0.0, 0.02))
            points.append(
                LiDARPoint(
                    angle=angle,
                    distance=round(noisy_distance, 4),
                    timestamp=now,
                    valid=True,
                )
            )

        frame = ScanFrame(
            scan_id=str(uuid.uuid4()),
            sequence_number=self._sequence_number,
            source_id=self.source_id,
            timestamp=now,
            points=points,
        )
        self._sequence_number += 1
        return frame
