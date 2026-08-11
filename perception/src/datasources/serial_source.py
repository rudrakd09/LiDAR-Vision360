"""Placeholder for the future real-hardware LiDAR data source (Phase 16).

This class deliberately does NOT implement a wire protocol. The STM32 <-> PC framing/packet
format has not been defined yet, and PROJECT_SPECIFICATION.md explicitly instructs against
inventing it ahead of the real hardware spec. This stub exists only so that:

1. The shape of the future integration is visible in the codebase now (constructor args,
   where it plugs into `LiDARDataSource`).
2. Code that type-hints against `LiDARDataSource` already works with either implementation
   without changes once this class is filled in.

When the STM32 UART protocol is defined, implement `connect`/`disconnect`/`read_scan` here:
open the serial port in `connect`, parse whatever framing the firmware sends in `read_scan`,
and construct `LiDARPoint`/`ScanFrame` instances identical in shape to what
`SimulatedLiDARDataSource` produces today.
"""

from __future__ import annotations

from datasources.base import LiDARDataSource
from models.scan import ScanFrame


class SerialLiDARDataSource(LiDARDataSource):
    """Not yet implemented. Reserved for the STM32 UART hardware adapter (Phase 16)."""

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self._connected = False

    def connect(self) -> None:
        raise NotImplementedError(
            "SerialLiDARDataSource is a placeholder. The STM32 UART packet protocol is not yet "
            "defined (see PROJECT_SPECIFICATION.md, Phase 16). Implement connect() once it is."
        )

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def read_scan(self) -> ScanFrame:
        raise NotImplementedError(
            "SerialLiDARDataSource is a placeholder. Implement read_scan() once the STM32 UART "
            "packet protocol is defined (Phase 16)."
        )
