"""Bounded, drop-oldest outgoing message queue (Phase 12).

Backs each connected client's own outgoing buffer in `streaming.server` -- a slow/stalled client
must never make the perception pipeline (or any other client) wait, and must never grow without
bound. See docs/communication.md "Non-blocking design" / "Frame dropping".

Deliberately not `queue.Queue` (Python's own would either block on `put()` when full, or require
the caller to `get_nowait()` + catch `queue.Full` on every enqueue to implement the drop-oldest
policy manually anyway) -- a small, purpose-built class matching the exact "keep the newest,
evict the oldest first" semantics this project needs is simpler than layering that policy on top
of a general-purpose queue. Named `LatestFrameQueue`, and this module `latest_frame_queue.py`
rather than `queue.py`, specifically to read unambiguously next to the standard library's own
`queue` module -- not a namespace collision in Python 3's explicit-relative-import world, but a
clearer name regardless.
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Any


class LatestFrameQueue:
    """Thread-safe bounded queue: `put()` never blocks and never raises when full -- it evicts
    the oldest queued item to make room for the newest, per `Settings.
    streaming_max_outgoing_queue`. `maxsize` must be `>= 1`.
    """

    def __init__(self, maxsize: int) -> None:
        if maxsize < 1:
            raise ValueError(f"maxsize must be >= 1, got {maxsize}")
        self._maxsize = maxsize
        self._items: deque = deque()
        self._dropped_count = 0
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)

    def put(self, item: Any) -> None:
        """Enqueue `item`, evicting the oldest queued item first if already at `maxsize`. Never
        blocks, never raises."""
        with self._lock:
            if len(self._items) >= self._maxsize:
                self._items.popleft()
                self._dropped_count += 1
            self._items.append(item)
            self._not_empty.notify()

    def get(self, timeout: float | None = None) -> Any:
        """Block until an item is available (or `timeout` elapses), then return the oldest
        queued item. Returns `None` on timeout -- callers use this to periodically re-check a
        "still running" flag rather than blocking forever."""
        with self._not_empty:
            if not self._items:
                self._not_empty.wait(timeout=timeout)
            if not self._items:
                return None
            return self._items.popleft()

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)

    @property
    def dropped_count(self) -> int:
        """Cumulative number of items evicted (never delivered) since construction -- surfaced
        for logging/metrics (see docs/communication.md "Logging": `[STREAM] Frames dropped`)."""
        with self._lock:
            return self._dropped_count

    def clear(self) -> None:
        with self._lock:
            self._items.clear()
