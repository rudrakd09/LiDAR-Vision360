"""Hardware-mode edge loop: `ESP32Source` -> adapter -> `LiveState` -> streaming server.

This is the hardware-mode analogue of `scripts/serve_unity_bridge.py`'s simulation `run()`. It
reuses the exact same downstream: `streaming.PerceptionStreamServer` on `Settings.
streaming_json_port` (5006), which the cloud backend and Unity both connect to independently --
so Dashboard / Unity / PostgreSQL need no hardware-specific code at all.

Behaviour when the ESP32 link is not delivering fresh frames (disconnected / timeout / stale /
invalid): it publishes a throttled `SYSTEM_STATUS` ("hardware_unavailable") + `ERROR`
(`HARDWARE_DATA_UNAVAILABLE`, with the concrete reason) and **stops publishing perception
frames**, so nothing downstream keeps presenting the last hardware frame as current, and it
never substitutes simulation data.
"""

from __future__ import annotations

import time

from common.config import Settings, get_settings
from common.logging import get_logger
from streaming import PerceptionStreamServer
from streaming.protocol import (
    build_error_message,
    build_perception_frame_message,
    build_system_status_message,
)

from .errors import ESP32ConfigurationError
from .live_state_adapter import ProcessedFrameToLiveState
from .source import ESP32Source

logger = get_logger(__name__)

_UNAVAILABLE_ERROR_CODE = "HARDWARE_DATA_UNAVAILABLE"


class _Throttle:
    def __init__(self, interval_s: float) -> None:
        self._interval = interval_s
        self._last = 0.0

    def ready(self, now: float) -> bool:
        if now - self._last >= self._interval:
            self._last = now
            return True
        return False


def run_esp32_edge(
    settings: Settings | None = None,
    *,
    rate_hz: float = 10.0,
    json_server: PerceptionStreamServer | None = None,
    source: ESP32Source | None = None,
    adapter: ProcessedFrameToLiveState | None = None,
    max_iterations: int | None = None,
    stop_event=None,
    now_fn=time.time,
) -> None:
    """Run the hardware edge loop until interrupted (or `max_iterations`/`stop_event` for tests).

    `json_server` / `source` / `adapter` are injectable for tests; in normal use all are built
    from `settings`.
    """
    settings = settings or get_settings()
    period_s = 1.0 / rate_hz if rate_hz > 0 else 0.0

    owns_server = json_server is None
    json_server = json_server or PerceptionStreamServer(settings=settings)
    source = source or ESP32Source(settings)
    adapter = adapter or ProcessedFrameToLiveState(settings, session_id=source.session_id)

    # Optional, fully independent (requirement 8): a SOFTWARE MODEL of the STM32's OTHER output --
    # CAN -> Vehicle ECU -- fed the same received STM32ProcessedFrame. Off unless
    # LIDAR_CAN_OUTPUT_ENABLED=true; a CAN-model failure is caught here and never touches the
    # ESP32 -> LiveState path. In the real vehicle this logic runs on the STM32, not the Edge.
    can_output = _maybe_start_can_output(settings)

    unavailable_throttle = _Throttle(settings.streaming_heartbeat_interval_s)

    if owns_server:
        try:
            json_server.start()
        except OSError as e:
            logger.error(
                "[ESP32] Could not start the streaming server on %s:%d -- %s. Is another bridge "
                "already running? Stop it first.",
                settings.streaming_host, settings.streaming_json_port, e,
            )
            raise SystemExit(1) from e

    json_server.set_session(session_id=source.session_id, source_id=settings.hardware_source_id)
    json_server.publish(build_system_status_message(
        status="running", source_id=settings.hardware_source_id,
        scan_rate_hz=rate_hz, session_id=source.session_id,
    ))

    try:
        source.connect()
    except ESP32ConfigurationError as e:
        logger.error(
            "[ESP32] %s\n"
            "       DATA_SOURCE=hardware requires LIDAR_ESP32_TRANSPORT to be set. Use "
            "'mock' for the simulated ESP32 transport, or provide a real "
            "datasources.esp32.transport.ESP32Transport once the Wi-Fi protocol is specified. "
            "This mode NEVER falls back to simulation data.", e,
        )
        if owns_server:
            json_server.stop()
        raise SystemExit(2) from e

    logger.info("[ESP32] hardware edge loop started (target %.1f Hz). Ctrl+C to stop.", rate_hz)
    iterations = 0
    try:
        while True:
            if stop_event is not None and stop_event.is_set():
                break
            if max_iterations is not None and iterations >= max_iterations:
                break
            iterations += 1
            loop_start = now_fn()

            frame = source.poll()
            status = source.status

            # Session boundary: ESP32Source mints a new session_id on (re)connect. Rebuild the
            # adapter so track history / events / last-risk memory are cleared, and tell the
            # stream server so every downstream consumer resets too (existing behaviour).
            if status.session_id != adapter.session_id:
                logger.info("[ESP32] session change %s -> %s -- resetting edge state.",
                            adapter.session_id, status.session_id)
                adapter.reset(session_id=status.session_id)
                json_server.set_session(session_id=status.session_id, source_id=settings.hardware_source_id)

            # `poll()` only ever returns a frame that was fresh, in-order, and valid when read --
            # publish it unconditionally. `status.state` drives ONLY the unavailable branch.
            if frame is not None:
                # CAN -> Vehicle ECU model (independent of everything below). Isolated so a CAN
                # error can never disturb the ESP32 -> LiveState -> Dashboard/Unity path.
                if can_output is not None:
                    try:
                        can_output.submit(frame)
                        can_output.tick(loop_start)
                    except Exception:  # noqa: BLE001 -- CAN model must never affect the ESP32 path
                        logger.exception("[SIM-CAN] CAN output model error (ignored -- ESP32 path unaffected).")

                adapted = adapter.build(frame)
                message = build_perception_frame_message(
                    adapted.tracked_scan,
                    collision_assessment=adapted.collision_assessment,
                    occupancy_grid=None,
                    vehicle_state=adapted.vehicle_state,
                    clearance_assessment=adapted.clearance_assessment,
                    settings=settings,
                    session_id=adapter.session_id,
                    live_state=adapted.live_state,
                )
                json_server.publish(message)
            else:
                reason = status.reason or "no processed frames from ESP32"
                if unavailable_throttle.ready(loop_start):
                    logger.warning("[ESP32] HARDWARE DATA UNAVAILABLE -- %s (state=%s).", reason, status.state.value)
                    json_server.publish(build_system_status_message(
                        status="hardware_unavailable", source_id=settings.hardware_source_id,
                        scan_rate_hz=rate_hz, session_id=adapter.session_id,
                    ))
                    json_server.publish(build_error_message(
                        code=_UNAVAILABLE_ERROR_CODE, message=reason,
                        session_id=adapter.session_id, source_id=settings.hardware_source_id,
                    ))

            if period_s > 0:
                elapsed = now_fn() - loop_start
                remaining = period_s - elapsed
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        logger.info("[ESP32] stopping (%d iteration(s), %d frame(s) forwarded).", iterations, json_server.frames_sent)
    finally:
        source.disconnect()
        if can_output is not None:
            try:
                can_output.stop()
            except Exception:  # noqa: BLE001
                pass
        if owns_server:
            json_server.stop()


def _maybe_start_can_output(settings: Settings):
    """Build + start a `can_output.STM32CANOutput` iff `settings.can_output_enabled`. Returns
    `None` otherwise (and on any error -- the CAN model is strictly optional)."""
    if not getattr(settings, "can_output_enabled", False):
        return None
    try:
        from can_output import build_can_output

        out = build_can_output(settings)
        out.start()
        logger.info("[SIM-CAN] STM32->ECU CAN output MODEL attached (backend=%s). NOT a real bus.",
                    getattr(settings, "can_output_backend", "mock"))
        return out
    except Exception:  # noqa: BLE001
        logger.exception("[SIM-CAN] could not start the CAN output model -- continuing without it.")
        return None


__all__ = ["run_esp32_edge"]
