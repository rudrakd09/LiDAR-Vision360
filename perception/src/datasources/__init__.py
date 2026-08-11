"""LiDAR data-source abstraction layer (Phase 16 groundwork, built now per Phase 0).

`LiDARDataSource` is the seam between "where the data comes from" (simulator today, STM32 UART
later) and the rest of the perception pipeline, which only ever depends on this interface.
"""

from .base import LiDARDataSource
from .serial_source import SerialLiDARDataSource
from .simulated import SimulatedLiDARDataSource

__all__ = ["LiDARDataSource", "SimulatedLiDARDataSource", "SerialLiDARDataSource"]
