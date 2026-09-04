"""ESP32 USB-serial raw-measurement data source (`DATA_SOURCE=esp32_serial`).

The in-service hardware path for this project::

    LiDAR -> STM32 -> ESP32 -> USB serial -> Windows Edge PC -> full perception pipeline

The ESP32 emits one ASCII measurement per line -- ``A:45 , D:1200`` (angle in degrees, distance
in millimetres) -- and the Edge PC does ALL the perception: parsing, validation, polar->Cartesian
conversion, 360 degree frame assembly, filtering, clustering, classification, tracking, clearance,
collision/TTC and scenario analysis.

Four single-purpose modules, each independently testable:

* `parser.MeasurementParser`   -- one line -> a validated `RawMeasurement` (or a counted rejection).
* `frame_builder.ScanFrameBuilder` -- a measurement stream -> complete `ScanFrame`s, via angle-wrap detection.
* `reader.SerialLineReader`    -- the COM port -> text lines, on a background thread, with reconnect.
* `source.ESP32SerialSource`   -- the three above, as one `datasources.SensorSource`.

Nothing downstream of `ScanFrame` knows this package exists -- that is what makes the same
pipeline run unchanged against simulation and against real hardware.
"""

from .errors import (
    ESP32SerialConfigurationError,
    ESP32SerialConnectionError,
    ESP32SerialError,
    ESP32SerialTimeoutError,
)
from .frame_builder import FrameBuilderStats, ScanFrameBuilder
from .parser import MeasurementParser, ParserStats, RawMeasurement
from .reader import ReaderStats, SerialLineReader
from .source import ESP32SerialSource

__all__ = [
    "ESP32SerialSource",
    "MeasurementParser",
    "ParserStats",
    "RawMeasurement",
    "ScanFrameBuilder",
    "FrameBuilderStats",
    "SerialLineReader",
    "ReaderStats",
    "ESP32SerialError",
    "ESP32SerialConfigurationError",
    "ESP32SerialConnectionError",
    "ESP32SerialTimeoutError",
]
