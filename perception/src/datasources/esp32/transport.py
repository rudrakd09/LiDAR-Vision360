"""Replaceable transport abstraction for the STM32 -> ESP32 -> Wi-Fi -> Edge link.

    ESP32  ->  ESP32Transport  ->  ESP32Source

`ESP32Transport` delivers **one processed-frame message worth of bytes at a time** (not a raw
byte stream), because *framing* -- length-prefix? delimiter? one datagram per frame? a WebSocket
message boundary? -- is exactly one of the things the hardware team has not specified. So the
transport owns framing, and `ESP32Source` + the Phase-2 deserializer
(`datasources.stm32.processed`) turn those bytes into a validated
`models.stm32_processed.STM32ProcessedFrame`. When the real Wi-Fi protocol is known, add a new
`ESP32Transport` subclass; nothing else changes.

**No real transport is implemented here** -- no IP, port, MQTT topic, TCP/UDP/WebSocket protocol,
or packet structure is invented. Two development transports ship:

* `MockESP32Transport` -- an in-memory queue you push frames/bytes/errors into. For unit tests
  and the SIMULATED ESP32 TRANSPORT end-to-end test.
* `ScriptedScenarioESP32Transport` -- emits a deterministic scripted "approaching vehicle"
  sequence of valid `STM32ProcessedFrame`s, so `ESP32_TRANSPORT=mock` gives a runnable demo of
  the hardware code path. It logs loudly, on every `open()`, that it is NOT real hardware.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections import deque
from typing import Iterable

from common.config import Settings, get_settings
from common.logging import get_logger
from models.clearance import ClearanceDirection, ClearanceState
from models.collision import RiskLevel, VehicleState
from models.mapping import VehiclePose
from models.objects import ObjectClassification, Point2D, Velocity2D
from models.stm32_processed import (
    STM32ChannelHealth,
    STM32Clearance,
    STM32FrameMetadata,
    STM32ProcessedFrame,
    STM32Risk,
    STM32SensorStatus,
    STM32SourceChannel,
    STM32SystemStatus,
    STM32TrackedObject,
)

from ..stm32.processed import JsonProcessedFrameCodec
from .errors import ESP32TransportError

logger = get_logger(__name__)


class ESP32Transport(ABC):
    """A replaceable message transport for the ESP32 link.

    Contract:
      * `open()` establishes the link. Raises `ESP32TransportError` on failure.
      * `close()` releases it. Safe to call when already closed.
      * `receive(timeout_s)` returns the bytes of ONE complete processed-frame message, or `None`
        if nothing arrived within `timeout_s` (a timeout is not an error). Raises
        `ESP32TransportError` if the link fails.
      * The bytes' encoding/framing are the transport's concern; `ESP32Source` only decodes them
        via a `datasources.stm32.processed` deserializer.
    """

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    def receive(self, timeout_s: float) -> bytes | None: ...


class MockESP32Transport(ESP32Transport):
    """In-memory transport for tests and the SIMULATED ESP32 TRANSPORT end-to-end test.

    Push items with `push_frame` / `push_bytes` / `push_error` / `push_timeout`; `receive()`
    pops them in order. An `Exception` item is raised (to simulate a link failure); a `None`
    item (from `push_timeout`) makes one `receive()` return `None`.
    """

    def __init__(self) -> None:
        self._queue: deque[bytes | BaseException | None] = deque()
        self._open = False
        self._codec = JsonProcessedFrameCodec()
        self.open_count = 0
        self.close_count = 0

    def open(self) -> None:
        self._open = True
        self.open_count += 1

    def close(self) -> None:
        self._open = False
        self.close_count += 1

    def is_open(self) -> bool:
        return self._open

    def receive(self, timeout_s: float) -> bytes | None:
        if not self._open:
            raise ESP32TransportError("MockESP32Transport.receive() called while closed.")
        if not self._queue:
            return None  # timeout -- nothing queued
        item = self._queue.popleft()
        if isinstance(item, BaseException):
            raise item
        return item

    # --- test helpers ---
    def push_frame(self, frame: STM32ProcessedFrame) -> None:
        self._queue.append(self._codec.encode(frame))

    def push_frames(self, frames: Iterable[STM32ProcessedFrame]) -> None:
        for f in frames:
            self.push_frame(f)

    def push_bytes(self, raw: bytes) -> None:
        self._queue.append(raw)

    def push_error(self, exc: BaseException | None = None) -> None:
        self._queue.append(exc if exc is not None else ESP32TransportError("simulated ESP32 link failure"))

    def push_timeout(self) -> None:
        self._queue.append(None)

    @property
    def pending(self) -> int:
        return len(self._queue)


def build_scripted_approaching_vehicle_frames(
    count: int,
    *,
    source_id: str = "stm32_hardware",
    start_distance_m: float = 22.0,
    approach_speed_mps: float = 2.0,
    scan_period_s: float = 0.1,
    start_timestamp: float | None = None,
) -> list[STM32ProcessedFrame]:
    """A deterministic scripted sequence: one `vehicle_like` object closing head-on, TTC
    shrinking, risk stepping SAFE -> WARNING -> CRITICAL, clearance tightening. Every frame is a
    fully valid Phase-2 `STM32ProcessedFrame`. **This is a scripted development fixture, not
    sensor data** -- see this module's docstring.
    """
    t0 = time.time() if start_timestamp is None else start_timestamp
    frames: list[STM32ProcessedFrame] = []
    for i in range(count):
        distance = max(0.5, start_distance_m - approach_speed_mps * scan_period_s * i)
        ttc = distance / approach_speed_mps
        if distance <= 3.0 or ttc <= 2.0:
            risk = RiskLevel.CRITICAL
        elif distance <= 8.0 or ttc <= 4.0:
            risk = RiskLevel.WARNING
        else:
            risk = RiskLevel.SAFE
        front_clear = max(0.1, distance - 2.5)
        clearance = STM32Clearance(
            front_m=round(front_clear, 3), rear_m=8.0, left_m=3.0, right_m=3.0,
            min_clearance_m=round(min(front_clear, 3.0), 3),
            min_direction=ClearanceDirection.FRONT if front_clear < 3.0 else ClearanceDirection.LEFT,
            status=(
                ClearanceState.CRITICAL if front_clear < 0.4
                else ClearanceState.LOW_CLEARANCE if front_clear < 0.8
                else ClearanceState.CAUTION if front_clear < 1.5
                else ClearanceState.SAFE
            ),
            corridor_width_m=6.0,
        )
        obj = STM32TrackedObject(
            track_id="stm32-track-1",
            object_type=ObjectClassification.VEHICLE_LIKE,
            position=Point2D(x=round(distance, 3), y=0.0),
            distance_m=round(distance, 3),
            relative_velocity=Velocity2D(vx=-approach_speed_mps, vy=0.0),
            confidence=0.9,
            ttc_s=round(ttc, 3),
            risk=risk,
            source=STM32SourceChannel.FUSED,
            in_projected_path=True,
        )
        frames.append(
            STM32ProcessedFrame(
                metadata=STM32FrameMetadata(
                    frame_id=i, sequence_number=i, timestamp=round(t0 + i * scan_period_s, 6),
                    source_id=source_id, active_object_count=1,
                ),
                sensor_status=STM32SensorStatus(
                    lidar=STM32ChannelHealth(connected=True, ok=True),
                    radar=STM32ChannelHealth(connected=True, ok=True),
                    fusion_active=True,
                ),
                vehicle_state=VehicleState(pose=VehiclePose(), speed_mps=0.0),
                objects=[obj],
                clearance=clearance,
                risk=STM32Risk(
                    overall_risk=risk,
                    most_critical_track_id="stm32-track-1",
                    collision_predicted=risk is RiskLevel.CRITICAL,
                    predicted_collision_time_s=round(ttc, 3) if risk is RiskLevel.CRITICAL else None,
                    reason=[f"scripted approaching vehicle at {distance:.1f} m"],
                ),
                system_status=STM32SystemStatus(stm32_ok=True, processing_time_ms=5.0),
            )
        )
    return frames


class ScriptedScenarioESP32Transport(ESP32Transport):
    """Emits a scripted sequence of valid `STM32ProcessedFrame`s, one frame per `receive()`
    call, looping when exhausted. Selected by `Settings.esp32_transport == "mock"`.

    The scene is chosen by `scenario` (falling back to `Settings.esp32_mock_scenario`):

    * `None` / `"approaching_vehicle"` -- the legacy `build_scripted_approaching_vehicle_frames`
      sequence (unchanged default).
    * `"realtime_arc"` / `"multi_object"` / `"tracking"` -- the Phase-10 MOCK STM32 HARDWARE
      OUTPUT scenes from `datasources.esp32.mock_stm32` (live, smooth-motion variants).

    Pass `frames=` to inject an explicit list (tests). **SIMULATED ESP32 TRANSPORT -- NOT REAL
    HARDWARE.** Logged loudly on every `open()`.
    """

    def __init__(
        self,
        settings: Settings | None = None,
        *,
        pace_s: float = 0.1,
        scenario: str | None = None,
        frames: list[STM32ProcessedFrame] | None = None,
    ) -> None:
        self._settings = settings or get_settings()
        self._codec = JsonProcessedFrameCodec()
        self._open = False
        self._pace_s = pace_s
        self._last_emit = 0.0
        self._scenario = (scenario if scenario is not None else self._settings.esp32_mock_scenario) or "approaching_vehicle"
        if frames is not None:
            self._frames = frames
        elif self._scenario.strip().lower() in ("", "approaching_vehicle"):
            self._frames = build_scripted_approaching_vehicle_frames(
                max(1, self._settings.esp32_mock_frame_count),
                source_id=self._settings.hardware_source_id,
            )
        else:
            from .mock_stm32 import build_mock_stm32_frames

            self._frames = build_mock_stm32_frames(self._scenario, self._settings, live=True)
        self._index = 0

    def open(self) -> None:
        self._open = True
        self._index = 0
        self._last_emit = 0.0
        logger.warning(
            "[ESP32] *** SIMULATED ESP32 TRANSPORT (MOCK STM32 HARDWARE OUTPUT, scenario=%s, "
            "%d frames) -- NOT REAL HARDWARE. Selected by LIDAR_ESP32_TRANSPORT=mock. ***",
            self._scenario, len(self._frames),
        )

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def receive(self, timeout_s: float) -> bytes | None:
        if not self._open:
            raise ESP32TransportError("ScriptedScenarioESP32Transport.receive() called while closed.")
        now = time.time()
        if self._pace_s > 0 and now - self._last_emit < self._pace_s:
            return None  # not time for the next scripted frame yet -- a real link is paced too
        self._last_emit = now
        frame = self._frames[self._index % len(self._frames)]
        self._index += 1
        # Re-stamp frame_id/sequence/timestamp so a long run keeps advancing monotonically past
        # one script length, rather than resetting to 0 and looking like a new session.
        restamped = frame.model_copy(
            update={
                "metadata": frame.metadata.model_copy(
                    update={
                        "frame_id": self._index - 1,
                        "sequence_number": self._index - 1,
                        "timestamp": round(time.time(), 6),
                    }
                )
            }
        )
        return self._codec.encode(restamped)


def build_transport(settings: Settings) -> ESP32Transport:
    """Factory: resolve `Settings.esp32_transport` to a concrete `ESP32Transport`.

    * `"mock"` -> `ScriptedScenarioESP32Transport` (development only).
    * unset / `"<CONFIGURE>"` -> caller should raise `ESP32ConfigurationError` (handled by
      `ESP32Source.connect()`); this function raises `ValueError` so the source can convert it.
    * any other value -> `ValueError` -- real transports (tcp/udp/mqtt/websocket/...) are not
      implemented because the Wi-Fi protocol is unspecified.
    """
    name = (settings.esp32_transport or "").strip().lower()
    if name == "mock":
        return ScriptedScenarioESP32Transport(settings, scenario=settings.esp32_mock_scenario)
    if not name or name == "<configure>":
        raise ValueError("Settings.esp32_transport is not configured.")
    raise ValueError(
        f"Settings.esp32_transport={settings.esp32_transport!r} is not implemented. The ESP32 "
        f"Wi-Fi protocol is not specified yet; only 'mock' is available. Provide a real "
        f"datasources.esp32.transport.ESP32Transport subclass once the hardware spec exists."
    )


__all__ = [
    "ESP32Transport",
    "MockESP32Transport",
    "ScriptedScenarioESP32Transport",
    "build_scripted_approaching_vehicle_frames",
    "build_transport",
]
