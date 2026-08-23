"""Abstract LiDAR data-source interface -- the sensor input abstraction.

The perception pipeline must never depend on *where* scan frames come from. Every producer of
`ScanFrame`s - a simulator, a replay-from-file tool, or eventually the real STM32 UART link -
implements this interface, so the rest of the system (preprocessing onward) is written once and
works unchanged in simulation and on real hardware (Phase 16).

`SensorFrame`/`SensorSource` below are the vocabulary this abstraction is addressed by from the
composition-root side (`scripts/sensor_source.py`'s `DATA_SOURCE=simulation|hardware` switch,
`simulator.SimulatorSource`, `datasources.STM32Source`) -- deliberately plain aliases of
`ScanFrame`/`LiDARDataSource`, not a second parallel schema/interface. There is exactly one
canonical frame type and one canonical source contract; giving them an additional name lets
call sites read as "any SensorSource produces a SensorFrame" without duplicating the models
`Preprocessor.process()` and everything downstream already depend on (see docs/architecture.md
"Sensor source abstraction" and PROJECT_SPECIFICATION.md's "do not hard-code... network
addresses" -- the point of this seam is that the pipeline never needs to change to accept a new
concrete source).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterator

from models.scan import ScanFrame

# The canonical frame type produced by any sensor source, regardless of origin (simulator, STM32,
# a recorded replay, ...). An explicit alias of `ScanFrame` -- not a new model -- so there is
# exactly one schema and zero conversion/drift risk between "SensorFrame" and what
# `preprocessing.Preprocessor.process()` (and every other existing pipeline stage) already
# accepts unchanged.
SensorFrame = ScanFrame


class LiDARDataSource(ABC):
    """Common interface for anything that can produce a stream of `ScanFrame`s."""

    @abstractmethod
    def connect(self) -> None:
        """Open/initialize the underlying source (simulator state, serial port, socket, ...)."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release any resources acquired by `connect`."""

    @abstractmethod
    def read_scan(self) -> ScanFrame:
        """Block until one full 360° `ScanFrame` is available, and return it."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the source is currently connected/ready to produce scans."""

    def stream(self) -> Iterator[ScanFrame]:
        """Convenience generator yielding scans while the source remains connected."""
        while self.is_connected():
            yield self.read_scan()

    def __enter__(self) -> "LiDARDataSource":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()


# Preferred name for this same interface from the sensor-source-abstraction call sites (see
# module docstring) -- a plain alias, not a second parallel base class, so
# `issubclass(X, SensorSource)` and `issubclass(X, LiDARDataSource)` always agree and every
# existing `LiDARDataSource`-typed consumer/test keeps working completely unchanged.
SensorSource = LiDARDataSource
