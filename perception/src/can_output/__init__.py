"""STM32 -> Vehicle-ECU CAN output -- a SOFTWARE MODEL of the CAN transmit stage the STM32
firmware performs in the real vehicle.

    STM32 processing/fusion
        │
        ├──►  CAN → Vehicle ECU        (this package -- can_output)
        │
        └──►  ESP32 → Wi-Fi → Edge PC  (datasources.esp32 -- entirely separate)

Both consume the same Phase-2 `models.stm32_processed.STM32ProcessedFrame`. This package imports
**nothing** from `datasources.esp32`; the ESP32 path works identically whether or not CAN output
runs (requirement 8).

**No real CAN specification is invented.** Bitrate, CAN IDs, standard/extended, DLC, byte
layout, endianness, scaling, signed-ness, signal offsets/lengths, checksum choice, and
transmission periods are all `None` (PENDING) and reported by
`CANOutputConfig.pending_hardware_parameters()`. What is defined is the *software design*: the
proposed message catalogue (`build_default_message_catalog`), the semantic field mapping
(`can_output.payload`), a standard DBC-style bit-packer (`SignalPacker`, a mechanism -- used only
once the layout is specified), transmission control + error handling + health
(`STM32CANOutput`), and replaceable software transports (`MockCANTransport` /
`LoggingCANTransport`, both **SIMULATED CAN OUTPUT** -- no bus required).

Until the DBC exists, `STM32CANOutput` produces *unencoded* frames (semantic signal values, no
bytes) so the pipeline runs end to end; once the DBC is filled into `CANOutputConfig`, the same
frames come out bit-packed with real CAN IDs -- no redesign (requirement 11).

See docs/can-output.md.
"""

from __future__ import annotations

from .encoder import CANFrameEncoder, SignalPacker
from .errors import (
    CANBusError,
    CANConfigurationError,
    CANOutputError,
    CANTransmitError,
    CANValidationError,
)
from .frame import CANFrame
from .message_spec import (
    CANByteOrder,
    CANMessageSpec,
    CANOutputConfig,
    CANSignalScaling,
    CANSignalSpec,
    CANTransmissionMode,
    build_default_config,
    build_default_message_catalog,
    clearance_direction_code,
    clearance_state_code,
    object_type_code,
    risk_code,
)
from .output import CANOutputHealth, CANOutputState, STM32CANOutput
from .payload import (
    ObjectStatePayload,
    PerceptionHeaderPayload,
    ProcessedPerceptionPayloads,
    SafetyStatePayload,
    extract_payloads,
    track_id_hash,
    validate_header,
    validate_object_state,
    validate_safety_state,
)
from .transport import (
    CANTransport,
    CANTransportState,
    LoggingCANTransport,
    MockCANTransport,
    build_can_transport,
)


def build_can_output(settings, *, transport: CANTransport | None = None) -> STM32CANOutput:
    """Compose a `STM32CANOutput` from `Settings` -- the proposed message catalogue plus every
    `can_output_*` value from config (hardware values still PENDING unless the operator filled
    them in). `transport` overrides the `Settings.can_output_backend` choice (tests inject a
    `MockCANTransport`)."""
    config = build_default_config(
        bitrate_bps=getattr(settings, "can_output_bitrate_bps", None),
        interface=getattr(settings, "can_output_interface", None),
        default_cycle_time_ms=getattr(settings, "can_output_default_cycle_time_ms", None),
        sequence_modulus=getattr(settings, "can_output_sequence_modulus", None),
        max_transmit_rate_hz=getattr(settings, "can_output_max_transmit_rate_hz", 100.0),
        queue_max_frames=getattr(settings, "can_output_queue_max_frames", 64),
        require_fully_specified=getattr(settings, "can_output_require_fully_specified", False),
        bus_recovery_initial_backoff_s=getattr(settings, "can_output_bus_recovery_initial_backoff_s", 1.0),
        bus_recovery_max_backoff_s=getattr(settings, "can_output_bus_recovery_max_backoff_s", 10.0),
    )
    for m in config.messages:
        if getattr(settings, "can_output_checksum_algorithm", None):
            m.checksum_algorithm = settings.can_output_checksum_algorithm
    return STM32CANOutput(config, transport or build_can_transport(settings))


__all__ = [
    # errors
    "CANOutputError", "CANConfigurationError", "CANValidationError", "CANBusError", "CANTransmitError",
    # spec / config
    "CANSignalScaling", "CANSignalSpec", "CANMessageSpec", "CANOutputConfig",
    "CANByteOrder", "CANTransmissionMode",
    "build_default_message_catalog", "build_default_config",
    "object_type_code", "risk_code", "clearance_direction_code", "clearance_state_code",
    # payloads
    "PerceptionHeaderPayload", "ObjectStatePayload", "SafetyStatePayload", "ProcessedPerceptionPayloads",
    "extract_payloads", "track_id_hash",
    "validate_header", "validate_object_state", "validate_safety_state",
    # frame / encoder
    "CANFrame", "SignalPacker", "CANFrameEncoder",
    # transport
    "CANTransport", "CANTransportState", "MockCANTransport", "LoggingCANTransport", "build_can_transport",
    # controller
    "STM32CANOutput", "CANOutputState", "CANOutputHealth",
    # convenience
    "build_can_output",
]
