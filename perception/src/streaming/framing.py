"""TCP message framing for the structured JSON perception protocol (Phase 12).

**Framing choice: newline-delimited JSON** (one JSON object per line) -- matching the same
`StreamReader.ReadLine()`-friendly style the legacy raw `<START>`/`<END>` protocol and every
existing Unity script already use, so no new delimiter/length-prefix scheme has to be designed
and kept in sync between Python and C#. A JSON object's own `{...}` structure is unambiguous once
a full line is available; the newline only marks "a full line has arrived," it is not part of the
JSON payload itself. See docs/communication.md "Message framing" for the full reasoning,
including why length-prefixing (the other standard option) was considered and not chosen.

TCP is a byte stream, not a message stream: a single `recv()` may return less than one full line,
more than one full line, or a line split across two `recv()` calls. `MessageFramer` handles all
three cases explicitly -- see `perception/tests/test_streaming_framing.py`, which exercises this
directly with synthetic byte chunks, no real socket required.
"""

from __future__ import annotations

import json


def encode_message(message: dict) -> bytes:
    """One JSON object, newline-terminated, UTF-8 encoded -- ready to `sendall()`."""
    return (json.dumps(message) + "\n").encode("utf-8")


class MessageFramer:
    """Accumulates raw bytes (however many `recv()` calls it takes) and yields complete, parsed
    JSON messages as soon as a full line is available.

    Stateful -- construct one instance per connection; never share a single instance across
    multiple sockets.
    """

    def __init__(self) -> None:
        self._buffer = b""

    def feed(self, chunk: bytes) -> list[dict]:
        """Add newly-received bytes; return every complete message now available -- zero, one,
        or many (a single `recv()` can straddle any number of message boundaries).

        Malformed JSON on an otherwise-complete line is silently skipped, not raised -- one
        corrupt line must never lose every well-formed message queued behind it in the same
        chunk, and framing itself has no opinion on how the caller should log a parse failure
        (see docs/communication.md "Data validation").
        """
        self._buffer += chunk
        messages: list[dict] = []

        while b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                messages.append(json.loads(line))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

        return messages

    def pending_bytes(self) -> int:
        """How many bytes are currently buffered, waiting for a terminating newline -- exposed so
        a caller can defend against an unbounded partial line from a misbehaving peer (see
        docs/communication.md "Security / validation": `streaming_max_message_bytes`). Framing
        itself does not enforce this limit -- it has no `Settings` dependency and no opinion on
        what "too large" means; the caller (`streaming.server`) does."""
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer = b""
