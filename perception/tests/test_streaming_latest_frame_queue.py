"""Tests for streaming.latest_frame_queue.LatestFrameQueue: bounded, drop-oldest, non-blocking
put(), thread-safety."""

import threading
import time

import pytest

from streaming.latest_frame_queue import LatestFrameQueue


class TestBasicPutGet:
    def test_put_then_get_returns_the_item(self):
        q = LatestFrameQueue(maxsize=3)
        q.put("a")
        assert q.get(timeout=0.1) == "a"

    def test_fifo_order_when_not_overflowing(self):
        q = LatestFrameQueue(maxsize=3)
        q.put("a")
        q.put("b")
        q.put("c")
        assert [q.get(timeout=0.1), q.get(timeout=0.1), q.get(timeout=0.1)] == ["a", "b", "c"]

    def test_get_on_empty_queue_times_out_to_none(self):
        q = LatestFrameQueue(maxsize=3)
        start = time.perf_counter()
        result = q.get(timeout=0.2)
        elapsed = time.perf_counter() - start
        assert result is None
        assert elapsed >= 0.15  # actually waited, not an immediate return


class TestBoundedDropOldest:
    def test_overflow_drops_oldest_keeps_newest(self):
        q = LatestFrameQueue(maxsize=2)
        q.put("a")
        q.put("b")
        q.put("c")  # should evict "a"
        assert [q.get(timeout=0.1), q.get(timeout=0.1)] == ["b", "c"]

    def test_never_exceeds_maxsize(self):
        q = LatestFrameQueue(maxsize=3)
        for i in range(100):
            q.put(i)
        assert len(q) == 3

    def test_dropped_count_tracks_evictions(self):
        q = LatestFrameQueue(maxsize=2)
        for i in range(10):
            q.put(i)
        assert q.dropped_count == 8  # 10 puts, capacity 2 -> 8 evicted

    def test_dropped_count_zero_when_never_overflowed(self):
        q = LatestFrameQueue(maxsize=5)
        q.put("a")
        q.put("b")
        assert q.dropped_count == 0


class TestMaxsizeValidation:
    def test_maxsize_zero_rejected(self):
        with pytest.raises(ValueError):
            LatestFrameQueue(maxsize=0)

    def test_maxsize_negative_rejected(self):
        with pytest.raises(ValueError):
            LatestFrameQueue(maxsize=-1)

    def test_maxsize_one_is_valid(self):
        q = LatestFrameQueue(maxsize=1)
        q.put("a")
        q.put("b")
        assert len(q) == 1
        assert q.get(timeout=0.1) == "b"


class TestClear:
    def test_clear_empties_the_queue(self):
        q = LatestFrameQueue(maxsize=5)
        q.put("a")
        q.put("b")
        q.clear()
        assert len(q) == 0
        assert q.get(timeout=0.1) is None


class TestNonBlockingPut:
    def test_put_never_blocks_even_when_full(self):
        q = LatestFrameQueue(maxsize=1)
        q.put("a")  # queue now full, nothing ever consuming it
        start = time.perf_counter()
        for i in range(1000):
            q.put(i)
        elapsed = time.perf_counter() - start
        assert elapsed < 0.5  # 1000 puts against a full, never-drained queue must still be fast


class TestThreadSafety:
    def test_concurrent_puts_from_multiple_threads_do_not_corrupt_state(self):
        q = LatestFrameQueue(maxsize=10)

        def producer(start):
            for i in range(start, start + 100):
                q.put(i)

        threads = [threading.Thread(target=producer, args=(t * 100,)) for t in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(q) == 10  # never exceeds maxsize even under concurrent access
        assert q.dropped_count == 400 - 10

    def test_get_blocks_until_a_concurrent_put_arrives(self):
        q = LatestFrameQueue(maxsize=3)
        result = {}

        def consumer():
            result["value"] = q.get(timeout=2.0)

        t = threading.Thread(target=consumer)
        t.start()
        time.sleep(0.1)  # ensure the consumer is already waiting
        q.put("delivered")
        t.join(timeout=2.0)

        assert result["value"] == "delivered"
