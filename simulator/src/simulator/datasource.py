"""The full-featured simulated LiDAR data source: ray-casts a configured `Environment` through a
`LiDARModel`'s sampling pattern, applies a `NoiseModel`, and steps moving obstacles between scans.

Implements `datasources.LiDARDataSource` from the perception foundation, so anything written
against that interface (the future pipeline, `RecordedLiDARDataSource`, tests) works unchanged
regardless of which concrete data source is plugged in.
"""

from __future__ import annotations

import time
import uuid

from datasources.base import LiDARDataSource
from models.lidar import LiDARPoint
from models.scan import ScanFrame

from .environment import Environment
from .lidar_model import LiDARModel
from .noise import NoiseModel
from .vehicle import VehicleConfig


class SimulatedLiDARDataSource(LiDARDataSource):
    """Produces `ScanFrame`s by ray-casting `environment` from the vehicle's LiDAR mount point.

    Args:
        environment: obstacles to ray-cast against.
        vehicle: vehicle dimensions + LiDAR mount pose.
        lidar_model: angular sampling pattern, range limits, scan frequency.
        noise_model: applies distance noise / outliers / missing measurements per ray.
        source_id: identifier recorded on every `ScanFrame.source_id`.
        real_time: if True, `read_scan()` sleeps to pace calls at `lidar_model.scan_frequency_hz`
            (wall-clock real-time). If False (default), scans are produced as fast as possible --
            appropriate for tests, batch recording, and headless replay generation. Either way,
            obstacle motion between scans always advances by the model's nominal
            `1 / scan_frequency_hz`, so behavior is deterministic and independent of wall-clock
            jitter.
    """

    def __init__(
        self,
        environment: Environment,
        vehicle: VehicleConfig,
        lidar_model: LiDARModel,
        noise_model: NoiseModel,
        source_id: str = "simulated",
        real_time: bool = False,
    ) -> None:
        self.environment = environment
        self.vehicle = vehicle
        self.lidar_model = lidar_model
        self.noise_model = noise_model
        self.source_id = source_id
        self.real_time = real_time

        self._connected = False
        self._sequence_number = 0
        self._has_scanned = False
        self._last_wall_time: float | None = None

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def read_scan(self) -> ScanFrame:
        if not self._connected:
            raise RuntimeError("SimulatedLiDARDataSource.read_scan() called before connect().")

        dt = 1.0 / self.lidar_model.scan_frequency_hz
        if self._has_scanned:
            # The first scan reflects the environment's initial state; obstacles move between
            # subsequent scans by the nominal inter-scan interval.
            self.environment.step(dt)
        self._has_scanned = True

        if self.real_time:
            self._pace_real_time(dt)

        now = time.time()
        origin = self.vehicle.lidar_origin
        orientation = self.vehicle.lidar_orientation_deg
        range_min, range_max = self.lidar_model.range_min_m, self.lidar_model.range_max_m

        points: list[LiDARPoint] = []
        for sensor_angle in self.lidar_model.angles_deg():
            world_angle = (sensor_angle + orientation) % 360.0
            true_d = self.environment.true_distance(origin, world_angle, range_max)
            distance, valid = self.noise_model.apply(true_d, range_min, range_max)
            points.append(
                LiDARPoint(angle=sensor_angle, distance=round(distance, 4), timestamp=now, valid=valid)
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

    def _pace_real_time(self, dt: float) -> None:
        now = time.time()
        if self._last_wall_time is not None:
            elapsed = now - self._last_wall_time
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)
        self._last_wall_time = time.time()
