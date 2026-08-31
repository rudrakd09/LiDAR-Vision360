"""ESP32 gateway: the STM32 -> ESP32 -> Wi-Fi -> Edge PC link that delivers ALREADY-PROCESSED
perception frames (Phase 2 `models.stm32_processed.STM32ProcessedFrame`), not raw sensor data.

    ESP32 --(ESP32Transport)--> ESP32Source --STM32ProcessedFrame--> ProcessedFrameToLiveState
        --> LiveState --> streaming.PerceptionStreamServer (5006) --> Dashboard / Unity / PostgreSQL

Selected by `Settings.data_source == "hardware"` (`scripts/serve_unity_bridge.py` dispatches to
`edge_runner.run_esp32_edge`). Simulation mode (`data_source == "simulation"`) is unchanged.

* `transport.ESP32Transport` -- replaceable message transport. No real Wi-Fi protocol is invented
  here; `MockESP32Transport` (tests) and `ScriptedScenarioESP32Transport` (`ESP32_TRANSPORT=mock`,
  a clearly-labelled development aid) ship. A real transport is a drop-in subclass.
* `source.ESP32Source` -- connection management, timeout, reconnect-with-backoff, heartbeat,
  stale-frame detection, sequence-gap / dropped-frame tracking, malformed-frame rejection.
  Never fabricates a frame; reports a concrete reason when data is unavailable.
* `live_state_adapter.ProcessedFrameToLiveState` -- reshapes one `STM32ProcessedFrame` into the
  canonical `TrackedScan` / `CollisionAssessment` / `ClearanceAssessment` and runs the existing
  `pipeline.LiveStateBuilder` (session identity, track history, events, measured rate). No
  perception is recomputed.
* `edge_runner.run_esp32_edge` -- the hardware-mode loop (analogue of the simulation bridge's
  `run()`).

See docs/esp32-integration.md.
"""

from __future__ import annotations

from .errors import (
    ESP32ConfigurationError,
    ESP32ConnectionError,
    ESP32Error,
    ESP32TransportError,
)
from .live_state_adapter import (
    AdaptedFrame,
    ProcessedFrameToLiveState,
    processed_frame_sensor_status,
    processed_frame_to_clearance_assessment,
    processed_frame_to_collision_assessment,
    processed_frame_to_tracked_scan,
)
from .mock_stm32 import (
    MOCK_STM32_SCENARIOS,
    MockObjectState,
    build_mock_processed_frame,
    build_mock_stm32_frames,
    build_multi_object_frames,
    build_realtime_arc_frames,
    build_tracking_frames,
)
from .source import ESP32Source, ESP32SourceState, ESP32SourceStatus, ProcessedFrameSource
from .transport import (
    ESP32Transport,
    MockESP32Transport,
    ScriptedScenarioESP32Transport,
    build_scripted_approaching_vehicle_frames,
    build_transport,
)
from .edge_runner import run_esp32_edge

__all__ = [
    # errors
    "ESP32Error",
    "ESP32ConfigurationError",
    "ESP32ConnectionError",
    "ESP32TransportError",
    # transport
    "ESP32Transport",
    "MockESP32Transport",
    "ScriptedScenarioESP32Transport",
    "build_scripted_approaching_vehicle_frames",
    "build_transport",
    # mock STM32 hardware output (Phase 10)
    "MOCK_STM32_SCENARIOS",
    "MockObjectState",
    "build_mock_processed_frame",
    "build_mock_stm32_frames",
    "build_realtime_arc_frames",
    "build_multi_object_frames",
    "build_tracking_frames",
    # source
    "ESP32Source",
    "ESP32SourceState",
    "ESP32SourceStatus",
    "ProcessedFrameSource",
    # adapter
    "AdaptedFrame",
    "ProcessedFrameToLiveState",
    "processed_frame_to_tracked_scan",
    "processed_frame_to_collision_assessment",
    "processed_frame_to_clearance_assessment",
    "processed_frame_sensor_status",
    # runner
    "run_esp32_edge",
]
