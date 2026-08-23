"""Generic byte-stream framing: turns a raw, continuous byte stream (as read off a serial port)
into discrete frame payloads, using whichever of the two standard framing conventions
configuration selects -- delimited (start/end marker) or fixed-length. Neither the marker bytes
nor the frame length are guessed here; only the two *mechanisms* for finding frame boundaries are
implemented, generically, so no code changes are needed once the real values are known -- only
configuration (`Settings.stm32_frame_start_marker`/`stm32_frame_end_marker`/
`stm32_frame_length_bytes`).
"""

from __future__ import annotations

from .errors import STM32ConfigurationError


class FrameCodec:
    """Stateful byte-stream framer: `feed()` bytes as they arrive from the transport, `
    extract_frames()` to pull out whatever complete frames are currently available. Partial data
    is retained across calls -- a frame split across two serial reads is handled transparently.

    Exactly one framing mode must be configured:
      - **Delimited**: `start_marker` required, `end_marker` optional (if omitted, a frame runs
        from one `start_marker` to the byte just before the next `start_marker`).
      - **Fixed-length**: `frame_length` required (bytes per frame, no delimiters).
    """

    def __init__(
        self,
        *,
        start_marker: bytes | None = None,
        end_marker: bytes | None = None,
        frame_length: int | None = None,
    ) -> None:
        if frame_length is not None and start_marker is not None:
            raise STM32ConfigurationError(
                "STM32 protocol configuration incomplete: frame format is ambiguous -- both a "
                "start marker and a fixed frame length were given; exactly one framing mode "
                "(delimited or fixed-length) is required."
            )
        if frame_length is None and start_marker is None:
            raise STM32ConfigurationError(
                "STM32 protocol configuration incomplete: frame format (start/end marker or "
                "fixed frame length) required."
            )
        if frame_length is not None and frame_length <= 0:
            raise STM32ConfigurationError("STM32 protocol configuration incomplete: frame length must be a positive number of bytes.")

        self.start_marker = start_marker
        self.end_marker = end_marker
        self.frame_length = frame_length
        self._buffer = bytearray()

    @property
    def is_fixed_length(self) -> bool:
        return self.frame_length is not None

    def feed(self, data: bytes) -> None:
        """Append newly-read bytes to the internal buffer."""
        self._buffer.extend(data)

    def buffered_byte_count(self) -> int:
        return len(self._buffer)

    def extract_frames(self) -> list[bytes]:
        """Pulls out every complete frame currently in the buffer (in order), leaving any
        trailing partial frame buffered for the next `feed()`. Returns `[]` if nothing complete is
        available yet."""
        if self.is_fixed_length:
            return self._extract_fixed_length()
        return self._extract_delimited()

    def _extract_fixed_length(self) -> list[bytes]:
        frames: list[bytes] = []
        n = self.frame_length
        while len(self._buffer) >= n:
            frames.append(bytes(self._buffer[:n]))
            del self._buffer[:n]
        return frames

    def _extract_delimited(self) -> list[bytes]:
        frames: list[bytes] = []
        start = bytes(self.start_marker)
        end = bytes(self.end_marker) if self.end_marker is not None else None

        while True:
            first = self._buffer.find(start)
            if first < 0:
                # No start marker at all yet -- discard any leading noise so the buffer doesn't
                # grow unbounded while waiting for one.
                self._buffer.clear()
                return frames
            if first > 0:
                # Drop noise before the first start marker.
                del self._buffer[:first]

            if end is not None:
                end_pos = self._buffer.find(end, len(start))
                if end_pos < 0:
                    return frames  # frame not complete yet
                frame_end = end_pos + len(end)
                frames.append(bytes(self._buffer[:frame_end]))
                del self._buffer[:frame_end]
            else:
                # No end marker -- a frame runs up to (not including) the NEXT start marker, so
                # we need to see that next marker before this frame is known to be complete.
                next_start = self._buffer.find(start, len(start))
                if next_start < 0:
                    return frames  # next frame hasn't started yet -- this one isn't complete
                frames.append(bytes(self._buffer[:next_start]))
                del self._buffer[:next_start]
