"""Connection lifecycle for a `Transport`: connect, disconnect, and reconnect-with-backoff.

Owns exactly the "is the link up, and if not, how do we get it back" concern -- framing,
validation, and parsing are all layered on top of this in `stm32_source.STM32Source`. Kept
separate so reconnect/backoff behavior is independently unit-testable against a fake `Transport`
(see `test_stm32_connection_manager.py`) without a real serial port.
"""

from __future__ import annotations

import time

from .errors import STM32ConnectionError
from .transport import Transport


class STM32ConnectionManager:
    def __init__(
        self,
        transport: Transport,
        *,
        connect_timeout_s: float = 5.0,
        reconnect_initial_backoff_s: float = 1.0,
        reconnect_max_backoff_s: float = 10.0,
        max_reconnect_attempts: int | None = None,
        sleep_fn=time.sleep,
    ) -> None:
        self._transport = transport
        self.connect_timeout_s = connect_timeout_s
        self.reconnect_initial_backoff_s = reconnect_initial_backoff_s
        self.reconnect_max_backoff_s = reconnect_max_backoff_s
        self.max_reconnect_attempts = max_reconnect_attempts
        self._sleep = sleep_fn

    def connect(self) -> None:
        """One connection attempt. Raises `STM32ConnectionError` on failure -- no retrying here;
        `reconnect()` is the retrying entry point, used once a previously-working connection is
        lost."""
        self._transport.open()

    def disconnect(self) -> None:
        self._transport.close()

    def is_connected(self) -> bool:
        return self._transport.is_open()

    def read(self, size: int) -> bytes:
        return self._transport.read(size)

    def write(self, data: bytes) -> int:
        return self._transport.write(data)

    def reconnect(self, on_attempt=None) -> None:
        """Closes the transport (if open) and retries `connect()` with exponential backoff,
        capped at `reconnect_max_backoff_s`, up to `max_reconnect_attempts` (`None` = retry
        forever). Raises `STM32ConnectionError` only once attempts are exhausted.

        `on_attempt(attempt_number, error)` is called after each failed attempt (before sleeping)
        -- used by `STM32Source` to update health/logging without this class needing to know
        about `HealthMonitor` directly.
        """
        self._transport.close()

        backoff = self.reconnect_initial_backoff_s
        attempt = 0
        last_error: Exception | None = None
        while self.max_reconnect_attempts is None or attempt < self.max_reconnect_attempts:
            attempt += 1
            try:
                self._transport.open()
                return
            except STM32ConnectionError as e:
                last_error = e
                if on_attempt is not None:
                    on_attempt(attempt, e)
                if self.max_reconnect_attempts is not None and attempt >= self.max_reconnect_attempts:
                    break
                self._sleep(backoff)
                backoff = min(backoff * 2, self.reconnect_max_backoff_s)

        raise STM32ConnectionError(
            f"Reconnection failed after {attempt} attempt(s); giving up "
            f"(max_reconnect_attempts={self.max_reconnect_attempts}). Last error: {last_error}"
        )
