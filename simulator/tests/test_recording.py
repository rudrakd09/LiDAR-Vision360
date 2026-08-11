"""Tests for simulator.recording: saving scans to .jsonl and replaying them."""

import pytest

from models import LiDARPoint, ScanFrame
from simulator.recording import (
    RecordedLiDARDataSource,
    iter_scans_jsonl,
    load_scans_jsonl,
    save_scan_jsonl,
    save_scans_jsonl,
)


def _make_frame(scan_id: str, sequence_number: int, num_points: int = 4) -> ScanFrame:
    points = [
        LiDARPoint(angle=float(i) * 10.0, distance=float(i) + 1.0, timestamp=1000.0 + i, valid=True)
        for i in range(num_points)
    ]
    return ScanFrame(scan_id=scan_id, sequence_number=sequence_number, source_id="unit-test", timestamp=1000.0, points=points)


class TestSaveAndLoad:
    def test_round_trip_preserves_frames(self, tmp_path):
        frames = [_make_frame("scan-a", 0), _make_frame("scan-b", 1)]
        path = tmp_path / "recording.jsonl"

        count = save_scans_jsonl(frames, path)

        assert count == 2
        loaded = load_scans_jsonl(path)
        assert loaded == frames

    def test_iter_matches_load(self, tmp_path):
        frames = [_make_frame("scan-a", 0), _make_frame("scan-b", 1), _make_frame("scan-c", 2)]
        path = tmp_path / "recording.jsonl"
        save_scans_jsonl(frames, path)

        streamed = list(iter_scans_jsonl(path))
        assert streamed == load_scans_jsonl(path)

    def test_append_mode_accumulates_frames(self, tmp_path):
        path = tmp_path / "recording.jsonl"
        save_scan_jsonl(_make_frame("scan-a", 0), path, append=False)
        save_scan_jsonl(_make_frame("scan-b", 1), path, append=True)

        loaded = load_scans_jsonl(path)
        assert [f.scan_id for f in loaded] == ["scan-a", "scan-b"]


class TestRecordedLiDARDataSource:
    def test_replays_frames_in_order(self, tmp_path):
        frames = [_make_frame("scan-a", 0), _make_frame("scan-b", 1)]
        path = tmp_path / "recording.jsonl"
        save_scans_jsonl(frames, path)

        source = RecordedLiDARDataSource(path)
        with source:
            first = source.read_scan()
            second = source.read_scan()

        assert first.scan_id == "scan-a"
        assert second.scan_id == "scan-b"

    def test_read_scan_before_connect_raises(self, tmp_path):
        path = tmp_path / "recording.jsonl"
        save_scans_jsonl([_make_frame("scan-a", 0)], path)
        source = RecordedLiDARDataSource(path)
        with pytest.raises(RuntimeError):
            source.read_scan()

    def test_exhausted_without_loop_stops(self, tmp_path):
        path = tmp_path / "recording.jsonl"
        save_scans_jsonl([_make_frame("scan-a", 0)], path)

        source = RecordedLiDARDataSource(path, loop=False)
        with source:
            source.read_scan()
            assert source.is_connected() is False
            with pytest.raises(StopIteration):
                source.read_scan()

    def test_loop_wraps_around(self, tmp_path):
        path = tmp_path / "recording.jsonl"
        save_scans_jsonl([_make_frame("scan-a", 0), _make_frame("scan-b", 1)], path)

        source = RecordedLiDARDataSource(path, loop=True)
        with source:
            ids = [source.read_scan().scan_id for _ in range(5)]
            assert source.is_connected()  # looping sources stay "connected" indefinitely

        assert ids == ["scan-a", "scan-b", "scan-a", "scan-b", "scan-a"]
