"""Sequence-number validation: detects dropped frames and (optionally) handles a wrapping
counter, for one logical channel (LiDAR or radar have independent sequence numbers on the wire,
so `STM32Source` owns one `SequenceValidator` per channel -- see `stm32_source.py`).

Mirrors the vocabulary `cloud/backend/src/backend/state.py` already uses for its own
duplicate/out-of-order tracking (`duplicate_or_out_of_order_dropped`), applied here at the Edge,
one layer earlier in the pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SequenceCheckResult:
    sequence_number: int
    is_first: bool  # no prior sequence number seen yet -- nothing to compare against
    dropped_count: int  # number of frames implied missing between the previous and this one (0 = none)
    is_duplicate_or_out_of_order: bool  # this sequence number is <= the last one seen (accounting for wraparound)


class SequenceValidator:
    """Tracks the last-seen sequence number for one channel.

    `modulus`: if the wire counter wraps (e.g. a `uint16` rolling over at 65536), pass that
    modulus so a wrap is recognized as "next" rather than a huge apparent drop or an
    out-of-order duplicate. `None` (default) means the counter never wraps.
    """

    def __init__(self, modulus: int | None = None) -> None:
        self.modulus = modulus
        self._last: int | None = None
        self.received_count = 0
        self.dropped_count = 0
        self.duplicate_or_out_of_order_count = 0

    def validate(self, sequence_number: int) -> SequenceCheckResult:
        self.received_count += 1

        if self._last is None:
            self._last = sequence_number
            return SequenceCheckResult(sequence_number, is_first=True, dropped_count=0, is_duplicate_or_out_of_order=False)

        expected_next = self._last + 1
        if self.modulus is not None:
            expected_next %= self.modulus

        if sequence_number == expected_next:
            self._last = sequence_number
            return SequenceCheckResult(sequence_number, is_first=False, dropped_count=0, is_duplicate_or_out_of_order=False)

        gap = self._gap_forward(self._last, sequence_number)
        if gap is not None and gap > 0:
            dropped = gap - 1
            self.dropped_count += dropped
            self._last = sequence_number
            return SequenceCheckResult(sequence_number, is_first=False, dropped_count=dropped, is_duplicate_or_out_of_order=False)

        # Not forward progress at all (including exact repeat) -- duplicate or reordered.
        self.duplicate_or_out_of_order_count += 1
        return SequenceCheckResult(sequence_number, is_first=False, dropped_count=0, is_duplicate_or_out_of_order=True)

    def _gap_forward(self, last: int, current: int) -> int | None:
        """How many steps forward from `last` to `current`, or `None` if `current` is not ahead
        of `last` (accounting for one wraparound if `modulus` is set).

        With a wrapping counter, "forward by a lot" and "backward by a little" are genuinely
        indistinguishable from the raw numbers alone -- e.g. with `modulus=65536`, `current =
        last - 1` and `current = last + 65535` compute to the same value. Standard heuristic:
        treat anything past half the modulus as backward/out-of-order rather than a near-total
        wraparound's worth of dropped frames, since a real drop count that large in one read is
        implausible.
        """
        if self.modulus is None:
            return current - last if current > last else None
        diff = (current - last) % self.modulus
        if diff == 0:
            return None
        if diff > self.modulus // 2:
            return None
        return diff
