"""Unit tests for `datasources.stm32.connection_manager.STM32ConnectionManager`, against an
in-memory fake `Transport` (never a real serial port). PARSER/UNIT TESTS ONLY -- not hardware
validation.
"""

import pytest

from datasources.stm32.connection_manager import STM32ConnectionManager
from datasources.stm32.errors import STM32ConnectionError
from datasources.stm32.transport import Transport


class FakeTransport(Transport):
    """In-memory `Transport` double: `fail_opens_remaining` controls how many `open()` calls fail
    before one succeeds, so reconnect/backoff logic can be tested deterministically."""

    def __init__(self, fail_opens_remaining: int = 0):
        self.fail_opens_remaining = fail_opens_remaining
        self.open_calls = 0
        self._open = False
        self.data = b""

    def open(self) -> None:
        self.open_calls += 1
        if self.fail_opens_remaining > 0:
            self.fail_opens_remaining -= 1
            raise STM32ConnectionError("simulated open failure")
        self._open = True

    def close(self) -> None:
        self._open = False

    def is_open(self) -> bool:
        return self._open

    def read(self, size: int) -> bytes:
        chunk, self.data = self.data[:size], self.data[size:]
        return chunk

    def write(self, data: bytes) -> int:
        return len(data)


class TestConnect:
    def test_connect_success(self):
        transport = FakeTransport()
        manager = STM32ConnectionManager(transport)
        manager.connect()
        assert manager.is_connected() is True

    def test_connect_failure_propagates(self):
        transport = FakeTransport(fail_opens_remaining=1)
        manager = STM32ConnectionManager(transport)
        with pytest.raises(STM32ConnectionError):
            manager.connect()
        assert manager.is_connected() is False


class TestReconnect:
    def test_reconnect_succeeds_after_transient_failures(self):
        transport = FakeTransport(fail_opens_remaining=2)
        sleeps: list[float] = []
        manager = STM32ConnectionManager(
            transport, reconnect_initial_backoff_s=0.01, reconnect_max_backoff_s=1.0, sleep_fn=sleeps.append
        )
        manager.reconnect()
        assert manager.is_connected() is True
        assert transport.open_calls == 3  # 2 failures + 1 success
        assert sleeps == [0.01, 0.02]  # exponential backoff between the two failed attempts

    def test_backoff_is_capped_at_max(self):
        transport = FakeTransport(fail_opens_remaining=5)
        sleeps: list[float] = []
        manager = STM32ConnectionManager(
            transport, reconnect_initial_backoff_s=1.0, reconnect_max_backoff_s=3.0, sleep_fn=sleeps.append
        )
        manager.reconnect()
        assert sleeps == [1.0, 2.0, 3.0, 3.0, 3.0]

    def test_reconnect_gives_up_after_max_attempts(self):
        transport = FakeTransport(fail_opens_remaining=10)
        manager = STM32ConnectionManager(
            transport, reconnect_initial_backoff_s=0.001, reconnect_max_backoff_s=0.001, max_reconnect_attempts=3, sleep_fn=lambda s: None
        )
        with pytest.raises(STM32ConnectionError):
            manager.reconnect()
        assert transport.open_calls == 3

    def test_on_attempt_callback_invoked_per_failure(self):
        transport = FakeTransport(fail_opens_remaining=2)
        calls: list[int] = []
        manager = STM32ConnectionManager(transport, reconnect_initial_backoff_s=0.001, sleep_fn=lambda s: None)
        manager.reconnect(on_attempt=lambda n, err: calls.append(n))
        assert calls == [1, 2]

    def test_reconnect_closes_existing_connection_first(self):
        transport = FakeTransport()
        manager = STM32ConnectionManager(transport)
        manager.connect()
        assert transport.is_open() is True
        manager.reconnect()
        # closed once (by reconnect) then reopened -- net effect is still connected
        assert manager.is_connected() is True
        assert transport.open_calls == 2


class TestReadWrite:
    def test_read_delegates_to_transport(self):
        transport = FakeTransport()
        transport.data = b"\x01\x02\x03"
        manager = STM32ConnectionManager(transport)
        manager.connect()
        assert manager.read(2) == b"\x01\x02"
        assert manager.read(2) == b"\x03"

    def test_write_delegates_to_transport(self):
        transport = FakeTransport()
        manager = STM32ConnectionManager(transport)
        manager.connect()
        assert manager.write(b"\x00\x01") == 2
