"""LiDAR data-source abstraction layer (Phase 16 groundwork, built now per Phase 0) -- the sensor
input abstraction.

`SensorSource` is the seam between "where the data comes from" (`simulator.SimulatorSource`
in simulation, `ESP32SerialSource` on the real USB-serial hardware link) and the rest of the perception
pipeline, which only ever depends on this interface and its `SensorFrame` output. `SensorSource`/
`SensorFrame` are plain aliases of the original `LiDARDataSource`/`ScanFrame` names (see
`datasources.base`'s own docstring) -- both names refer to exactly the same class/model, so
existing code written against either name keeps working unchanged.
"""

from .base import LiDARDataSource, SensorFrame, SensorSource
from .esp32_serial import ESP32SerialSource
from .serial_source import SerialLiDARDataSource
from .simulated import SimulatedLiDARDataSource
from .stm32_source import STM32Source

__all__ = [
    "LiDARDataSource",
    "SensorSource",
    "SensorFrame",
    "ESP32SerialSource",
    "SimulatedLiDARDataSource",
    "SerialLiDARDataSource",
    "STM32Source",
]
