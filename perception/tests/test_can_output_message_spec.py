"""Phase 5 -- CAN message/signal configuration (`can_output.message_spec`).

No CAN parameters are shipped: this asserts every hardware value is `None` (PENDING) in the
default catalogue and is surfaced by `pending_hardware_parameters()`, and that filling them in
flips `is_fully_specified` without any code change.
"""

from __future__ import annotations

import pytest

from can_output import (
    CANByteOrder,
    CANConfigurationError,
    CANMessageSpec,
    CANOutputConfig,
    CANSignalScaling,
    CANSignalSpec,
    CANTransmissionMode,
    build_default_config,
    build_default_message_catalog,
    clearance_direction_code,
    object_type_code,
    risk_code,
)
from can_output.message_spec import CLEARANCE_STATE_CODES, RISK_CODES
from models.clearance import ClearanceDirection
from models.collision import RiskLevel
from models.objects import ObjectClassification


class TestDefaultCatalogueShipsNoCANSpec:
    def test_three_conceptual_messages(self):
        cat = build_default_message_catalog()
        assert {m.name for m in cat} == {"PERCEPTION_HEADER", "OBJECT_STATE", "SAFETY_STATE"}

    def test_every_can_id_dlc_and_layout_is_pending(self):
        for m in build_default_message_catalog():
            assert m.can_id is None
            assert m.extended_id is None
            assert m.dlc is None
            assert not m.is_fully_specified
            for s in m.signals:
                assert s.start_bit is None
                assert s.length_bits is None
                assert s.byte_order is None
                if not (s.is_checksum or s.is_counter):
                    assert not s.scaling.is_specified

    def test_object_state_is_multiplexed_safety_is_periodic_and_event(self):
        cfg = build_default_config()
        assert cfg.message("OBJECT_STATE").multiplexed is True
        assert cfg.message("SAFETY_STATE").transmission_mode is CANTransmissionMode.PERIODIC_AND_EVENT

    def test_pending_hardware_parameters_lists_everything(self):
        cfg = build_default_config()
        pending = cfg.pending_hardware_parameters()
        assert "bitrate_bps" in pending["bus"]
        assert "interface" in pending["bus"]
        for msg in ("PERCEPTION_HEADER", "OBJECT_STATE", "SAFETY_STATE"):
            assert any(msg in k for k in pending), msg
            joined = " ".join(pending[msg])
            assert "can_id" in joined and "dlc" in joined and "start_bit" in joined

    def test_not_ready_for_hardware_by_default(self):
        assert build_default_config().is_ready_for_hardware is False


class TestFillingInTheSpec:
    def _fully_specify(self, m: CANMessageSpec, base_id: int) -> CANMessageSpec:
        m.can_id = base_id
        m.extended_id = False
        m.dlc = 8
        m.cycle_time_ms = 50.0
        bit = 0
        for s in m.signals:
            s.start_bit = bit
            s.length_bits = 4 if (s.is_counter or s.is_checksum) else 8
            s.byte_order = CANByteOrder.LITTLE_ENDIAN
            if not (s.is_counter or s.is_checksum):
                s.scaling = CANSignalScaling(scale=1.0, offset=0.0, signed=False)
            bit += 4 if (s.is_counter or s.is_checksum) else 8
        return m

    def test_is_fully_specified_flips_with_no_code_change(self):
        cfg = build_default_config()
        assert not cfg.is_ready_for_hardware
        for i, m in enumerate(cfg.messages):
            self._fully_specify(m, 0x100 + i)
        cfg.bitrate_bps = 500_000
        cfg.interface = "can0"
        assert cfg.is_ready_for_hardware is True
        assert cfg.pending_hardware_parameters() == {}

    def test_require_fully_specified_config_rejected_until_filled(self):
        from can_output import STM32CANOutput, MockCANTransport
        cfg = build_default_config(require_fully_specified=True)
        out = STM32CANOutput(cfg, MockCANTransport())
        with pytest.raises(CANConfigurationError, match="PENDING"):
            out.start()


class TestProposedCodes:
    def test_unknown_object_type_is_zero(self):
        assert object_type_code(ObjectClassification.UNKNOWN) == 0

    def test_all_enums_have_a_code(self):
        for c in ObjectClassification:
            assert isinstance(object_type_code(c), int)
        for r in RiskLevel:
            assert r in RISK_CODES
        for d in ClearanceDirection:
            assert isinstance(clearance_direction_code(d), int)
        assert len(CLEARANCE_STATE_CODES) == 4

    def test_risk_codes_ascend_by_severity(self):
        assert risk_code(RiskLevel.SAFE) < risk_code(RiskLevel.WARNING) < risk_code(RiskLevel.CRITICAL)


class TestOperationalGuards:
    def test_defaults_are_edge_side_not_hardware(self):
        cfg = CANOutputConfig()
        assert cfg.max_transmit_rate_hz == 100.0
        assert cfg.queue_max_frames == 64
        assert cfg.require_fully_specified is False
        # but the actual bus params are still unset
        assert cfg.bitrate_bps is None and cfg.interface is None
