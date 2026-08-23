"""Unit tests for `datasources.stm32.transport`. PARSER/UNIT TESTS ONLY -- not hardware
validation. `SerialTransport` is exercised only against a port that cannot possibly exist, to
prove it wraps a real connection failure correctly -- never against real hardware.
"""

import pytest

from datasources.stm32.errors import STM32ConnectionError
from datasources.stm32.transport import SerialTransport


class TestSerialTransportOpenFailure:
    def test_open_nonexistent_port_raises_stm32_connection_error(self):
        # A COM port number this high does not exist on any real machine -- this proves
        # SerialTransport wraps pyserial's own connection failure into STM32ConnectionError,
        # without requiring (or claiming) any actual hardware is attached.
        transport = SerialTransport(port="COM999", baudrate=115200, timeout_s=0.1)
        with pytest.raises(STM32ConnectionError):
            transport.open()

    def test_not_open_by_default(self):
        transport = SerialTransport(port="COM999", baudrate=115200, timeout_s=0.1)
        assert transport.is_open() is False

    def test_read_before_open_raises(self):
        transport = SerialTransport(port="COM999", baudrate=115200, timeout_s=0.1)
        with pytest.raises(STM32ConnectionError):
            transport.read(10)

    def test_write_before_open_raises(self):
        transport = SerialTransport(port="COM999", baudrate=115200, timeout_s=0.1)
        with pytest.raises(STM32ConnectionError):
            transport.write(b"\x00")

    def test_close_before_open_is_safe(self):
        transport = SerialTransport(port="COM999", baudrate=115200, timeout_s=0.1)
        transport.close()  # must not raise
        assert transport.is_open() is False
