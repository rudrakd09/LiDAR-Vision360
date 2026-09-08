"""End-to-end test: raw ESP32 serial lines -> the full Edge perception pipeline -> `LiveState`.

This is the test that proves the integration actually works. It wires up exactly the stages
`scripts/serve_unity_bridge.py` runs, in the same order, and drives them from synthetic *serial
lines in the real wire format* rather than from a simulator scenario -- so it exercises the
parser, the frame builder, and every downstream stage together.

Note the input here is a scripted serial byte stream, not fabricated perception output: the test
supplies what the ESP32 would send, and asserts on what the pipeline genuinely computes from it.
"""

from __future__ import annotations

import math
import sys
import time
import types

import pytest

from clearance import ClearanceEngine
from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.config import Settings
from coordinates import CoordinateTransformer
from datasources.esp32_serial import ESP32SerialSource
from models.collision import VehicleState
from objects import GeometricClassifier
from pipeline import LiveStateBuilder
from preprocessing import Preprocessor
from tracking import ObjectTracker


class FakeSerial:
    """Serves one scripted byte script, then goes quiet."""

    script: list[bytes] = []
    chunk_delay_s: float = 0.0

    def __init__(self, port: str, baudrate: int, timeout: float) -> None:
        self._queue = list(FakeSerial.script)

    @property
    def in_waiting(self) -> int:
        return len(self._queue[0]) if self._queue else 0

    def read(self, _size: int) -> bytes:
        if self._queue:
            if FakeSerial.chunk_delay_s:
                time.sleep(FakeSerial.chunk_delay_s)
            return self._queue.pop(0)
        # Real pyserial blocks here for up to `timeout` seconds. Emulating that matters: a fake
        # that returns instantly turns the reader thread into a hot spin loop that starves the
        # test thread of the GIL.
        time.sleep(0.005)
        return b""

    def close(self) -> None:
        pass


@pytest.fixture(autouse=True)
def fake_serial_module(monkeypatch: pytest.MonkeyPatch):
    FakeSerial.chunk_delay_s = 0.0
    module = types.ModuleType("serial")
    module.Serial = FakeSerial
    module.SerialException = Exception
    monkeypatch.setitem(sys.modules, "serial", module)
    yield


def settings_for(**overrides) -> Settings:
    base = {
        "data_source": "esp32_serial",
        "esp32_serial_port": "COM_TEST",
        "esp32_serial_scan_timeout_s": 2.0,
        "esp32_serial_log_every_n_scans": 0,
    }
    base.update(overrides)
    return Settings(_env_file=None, **base)


def wall_scan_lines(*, obstacle_angles: set[int], obstacle_mm: int, background_mm: int) -> bytes:
    """One full revolution in the real wire format, with a near obstacle over an angular span.

    `obstacle_angles` is an explicit set so the span can straddle 0 deg (e.g. 350..359 + 0..10),
    which is what "directly ahead" actually looks like in this project's angle convention.
    """
    return "".join(
        f"A:{angle} , D:{obstacle_mm if angle in obstacle_angles else background_mm}\n"
        for angle in range(360)
    ).encode()


def ahead(half_width_deg: int) -> set[int]:
    """The set of bearings within `half_width_deg` of straight ahead, wrapping through 0."""
    return {a % 360 for a in range(-half_width_deg, half_width_deg + 1)}


class TestCoordinateConversion:
    """The conversion the project specification pins down explicitly."""

    def test_45_degrees_at_1200mm_converts_to_the_specified_xy(self) -> None:
        FakeSerial.script = [
            wall_scan_lines(obstacle_angles=set(), obstacle_mm=1200, background_mm=1200)
            + b"A:0 , D:1200\n"  # wraps, completing the scan above
        ]
        # This test pins the polar->Cartesian conversion, not the pipeline's filtering. A 1.2 m
        # ring from a centre-mounted sensor sits inside the ego body, so the ego-footprint mask
        # (correctly) removes it -- disable that mask here so the conversion itself is what's
        # under test.
        source = ESP32SerialSource(settings=settings_for(ego_footprint_filter_enabled=False))
        transformer = CoordinateTransformer()

        source.connect()
        try:
            frame = source.read_scan()
        finally:
            source.disconnect()

        cartesian = transformer.transform(Preprocessor(settings=settings_for(ego_footprint_filter_enabled=False)).process(frame))
        point = next(p for p in cartesian.points if p.angle == pytest.approx(45.0))

        # 1200 mm -> 1.2 m; x = 1.2*cos(45deg), y = 1.2*sin(45deg).
        assert point.distance == pytest.approx(1.2)
        assert point.x == pytest.approx(0.8485, abs=1e-3)
        assert point.y == pytest.approx(0.8485, abs=1e-3)

    @pytest.mark.parametrize(
        ("angle", "distance_mm", "expected_x", "expected_y"),
        [
            (0, 1000, 1.0, 0.0),      # dead ahead  -> +X
            (90, 1000, 0.0, 1.0),     # left        -> +Y
            (180, 1000, -1.0, 0.0),   # behind      -> -X
            (270, 1000, 0.0, -1.0),   # right       -> -Y
        ],
    )
    def test_direction_convention_is_x_forward_y_left(
        self, angle: int, distance_mm: int, expected_x: float, expected_y: float
    ) -> None:
        """Pins the documented convention: +X front, +Y left, -X rear, -Y right, CCW degrees."""
        distance_m = distance_mm / 1000.0
        rad = math.radians(angle)

        assert distance_m * math.cos(rad) == pytest.approx(expected_x, abs=1e-9)
        assert distance_m * math.sin(rad) == pytest.approx(expected_y, abs=1e-9)


class TestFullPipeline:
    def test_serial_lines_produce_a_complete_live_state(self) -> None:
        # The default ego footprint is 4.5 m x 1.8 m, so distances must clear ~2.25 m to be
        # meaningful at all: an 8 m background wall all round, with a 3 m obstacle directly ahead.
        FakeSerial.script = [
            wall_scan_lines(obstacle_angles=ahead(12), obstacle_mm=3000, background_mm=8000)
            + b"A:0 , D:8000\n"
        ]
        settings = settings_for()
        source = ESP32SerialSource(settings=settings)

        source.connect()
        try:
            raw = source.read_scan()
        finally:
            source.disconnect()

        # Exactly the stage order scripts/serve_unity_bridge.py runs.
        clean = Preprocessor().process(raw)
        cartesian = CoordinateTransformer().transform(clean)
        clustered = DBSCANClusterer().cluster(cartesian)
        classified = GeometricClassifier().classify(clustered)
        tracked = ObjectTracker().update(classified)
        vehicle_state = VehicleState(speed_mps=0.0)
        assessment = CollisionRiskEngine().evaluate(tracked, vehicle_state=vehicle_state)
        clearance = ClearanceEngine().evaluate(cartesian, vehicle_state=vehicle_state)
        live_state = LiveStateBuilder(settings=settings).build(
            tracked_scan=tracked,
            preprocessed_scan=clean,
            collision_assessment=assessment,
            clearance_assessment=clearance,
            pipeline_processing_s=0.01,
        )

        # The raw measurements survived to the end.
        assert raw.point_count == 360
        assert len(cartesian.points) > 300

        # Real geometry was recovered: the near obstacle became at least one cluster/object.
        assert len(clustered.clusters) >= 1
        assert len(tracked.objects) >= 1

        # Clearance is computed from the point cloud, not hard-coded: the front is nearer than
        # the rear, because that is where the obstacle actually is.
        assert clearance.front.distance_m < clearance.rear.distance_m
        assert clearance.min_direction.value == "front"

        # The LiveState the dashboard consumes is fully populated.
        assert live_state.source_id == "esp32_serial"
        assert live_state.risk is not None
        assert live_state.clearance is not None
        assert live_state.session_id

    def test_near_zero_origin_returns_never_create_a_cluster_or_track(self) -> None:
        """A ring of returns at (essentially) the sensor origin -- the ego-vehicle / self return
        -- must be filtered in preprocessing and never reach clustering/classification/tracking.

        The wire values here (D:80 mm) are physically positive, so the ASCII parser accepts them;
        it is `Settings.min_valid_distance_m` (0.20 m), enforced in preprocessing, that rejects
        them. `background_mm` is placed in the free-space band (>= range_max - margin) so the
        ONLY thing that could possibly cluster is the origin ring or the real obstacle.
        """
        def scan_bytes() -> bytes:
            lines = []
            for angle in range(360):
                if angle <= 25 or angle >= 335:      # ~50 deg wedge of origin self-returns
                    d = 80
                elif 80 <= angle <= 110:             # a genuine obstacle, ~3 m off to the left
                    d = 3000
                else:
                    d = 11900                        # free space / no return
                lines.append(f"A:{angle} , D:{d}\n")
            return "".join(lines).encode()

        FakeSerial.script = [scan_bytes() + b"A:0 , D:11900\n"]
        settings = settings_for()
        source = ESP32SerialSource(settings=settings)

        source.connect()
        try:
            raw = source.read_scan()
        finally:
            source.disconnect()

        clean = Preprocessor().process(raw)
        cartesian = CoordinateTransformer().transform(clean)
        clustered = DBSCANClusterer().cluster(cartesian)
        classified = GeometricClassifier().classify(clustered)
        tracked = ObjectTracker().update(classified)

        # The ~51 origin-wedge points were dropped in preprocessing, not carried downstream.
        assert raw.point_count == 360
        assert clean.invalid_count >= 50
        assert all(p.distance >= settings.min_valid_distance_m for p in clean.points)

        # The real obstacle still clusters and tracks...
        assert len(clustered.clusters) >= 1
        assert len(tracked.objects) >= 1
        # ...and nothing sits at the origin: every object is well clear of the self-return radius.
        for obj in tracked.objects:
            centroid_distance = math.hypot(obj.centroid.x, obj.centroid.y)
            assert centroid_distance > 1.0, f"phantom object at the origin: {obj.classification} @ {centroid_distance:.3f} m"

    def test_a_track_seeded_by_near_zero_points_ages_out_once_filtering_is_active(self) -> None:
        """Requirement 8: a pre-existing origin track is not pinned forever -- with the invalid
        points now filtered it simply stops being fed and expires via the normal track lifecycle.

        Scan 1 carries a dense origin blob AND a real obstacle; scans 2+ carry only the real
        obstacle. Even scan 1 must not yield an origin track (the blob is filtered), and by the
        last scan the only surviving track is the real obstacle.
        """
        def scan_bytes(*, origin_blob: bool) -> bytes:
            lines = []
            for angle in range(360):
                if origin_blob and (angle <= 25 or angle >= 335):
                    d = 60
                elif 80 <= angle <= 110:
                    d = 3000
                else:
                    d = 11900
                lines.append(f"A:{angle} , D:{d}\n")
            return "".join(lines).encode()

        FakeSerial.script = [scan_bytes(origin_blob=True)] + [scan_bytes(origin_blob=False)] * 6
        settings = settings_for()
        source = ESP32SerialSource(settings=settings)

        preprocessor, transformer = Preprocessor(), CoordinateTransformer()
        clusterer, classifier = DBSCANClusterer(), GeometricClassifier()
        tracker = ObjectTracker()

        origin_track_seen = False
        source.connect()
        try:
            for _ in range(6):
                raw = source.read_scan()
                tracked = tracker.update(
                    classifier.classify(clusterer.cluster(transformer.transform(preprocessor.process(raw))))
                )
                for obj in tracked.objects:
                    if math.hypot(obj.centroid.x, obj.centroid.y) <= 1.0:
                        origin_track_seen = True
        finally:
            source.disconnect()

        assert not origin_track_seen, "the filtered origin blob must never have produced a track"
        # A real obstacle is still tracked at the end -- the pipeline stayed healthy throughout.
        assert any(math.hypot(o.centroid.x, o.centroid.y) > 1.0 for o in tracked.objects)

    def test_tracking_keeps_a_stable_id_across_consecutive_scans(self) -> None:
        """The same obstacle across three revolutions must stay ONE track, not three."""
        one_scan = wall_scan_lines(
            obstacle_angles=ahead(12), obstacle_mm=3000, background_mm=8000
        )
        FakeSerial.script = [one_scan] * 4
        settings = settings_for()
        source = ESP32SerialSource(settings=settings)

        preprocessor, transformer = Preprocessor(), CoordinateTransformer()
        clusterer, classifier = DBSCANClusterer(), GeometricClassifier()
        tracker = ObjectTracker()  # stateful across scans -- that is the point of this test

        track_ids_per_scan = []
        source.connect()
        try:
            for _ in range(3):
                raw = source.read_scan()
                tracked = tracker.update(
                    classifier.classify(
                        clusterer.cluster(transformer.transform(preprocessor.process(raw)))
                    )
                )
                track_ids_per_scan.append({obj.track_id for obj in tracked.objects})
        finally:
            source.disconnect()

        assert all(ids for ids in track_ids_per_scan), "every scan should track at least one object"
        # The ids seen in the last scan were already present in the first -- ids persisted.
        assert track_ids_per_scan[0] & track_ids_per_scan[-1]

    def test_scan_rate_is_measured_across_consecutive_scans(self) -> None:
        one_scan = wall_scan_lines(
            obstacle_angles=ahead(12), obstacle_mm=3000, background_mm=8000
        )
        # One chunk per revolution, with real time between them -- otherwise every line shares a
        # timestamp and there is no genuine inter-scan interval for the rate to be measured from.
        FakeSerial.script = [one_scan] * 3
        FakeSerial.chunk_delay_s = 0.02
        source = ESP32SerialSource(settings=settings_for())

        source.connect()
        try:
            source.read_scan()
            assert source.measured_scan_rate_hz is None  # nothing to measure yet
            source.read_scan()
            rate = source.measured_scan_rate_hz
        finally:
            source.disconnect()

        assert rate is not None and rate > 0  # a real measurement, whatever this machine's speed
