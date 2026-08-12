#!/usr/bin/env python
"""Phase 12 performance/load benchmark for the real-time streaming layer.

Measures (see docs/communication.md "Performance"):

- **Serialization time** and **message size** per scenario (raw-only, perception frame without
  points, with 360 points, with 10/50 objects, with an occupancy-map update, and a "maximum
  realistic frame" combining all of the above).
- **End-to-end latency** (avg/p95/max) via a real local TCP client that connects, reads, and
  parses messages on its own background thread -- mirroring `PerceptionTCPClient.cs`'s own
  newline-delimited-JSON-on-a-background-thread design closely enough to be a meaningful stand-in
  for it, since no Unity Editor is available in this environment (see docs/unity.md "Status").
- **Throughput and dropped-frame count** at 10/20/30 Hz target publish rates with increasing
  object counts (the "load test," per this phase's spec) -- to find where performance begins to
  degrade, not to pre-optimize before measuring.

**Local-clock assumption** (see docs/communication.md "Latency measurement"): sender and receiver
here are both this same Python process on the same machine, sharing one clock -- so subtracting
`transmission_timestamp` from arrival time is a valid, direct latency measurement with no clock-
sync concern. This would NOT hold across two different machines (e.g. Python on a PC, Unity on a
different device) without NTP or an explicit clock-offset calibration -- documented, not solved,
since this project's whole scope so far is a single local machine.

Usage:
    python scripts/benchmark_streaming.py
    python scripts/benchmark_streaming.py --objects 50 --duration 5
"""

from __future__ import annotations

import argparse
import json
import socket
import statistics
import threading
import time

import numpy as np

from common.config import Settings
from common.logging import get_logger, setup_logging
from models.collision import CollisionAssessment, CollisionRiskResult, RiskLevel, VehicleState
from models.lidar import CartesianPoint
from models.mapping import CellState, OccupancyGrid
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState, Velocity2D
from models.tracking import TrackedScan
from streaming.framing import MessageFramer, encode_message
from streaming.latest_frame_queue import LatestFrameQueue
from streaming.protocol import build_perception_frame_message
from streaming.server import PerceptionStreamServer, format_raw_scan

logger = get_logger(__name__)


# --- Synthetic fixture builders (deterministic, sized however a scenario needs) -----------------

def _object(i: int) -> DetectedObject:
    x, y = 3.0 + (i % 10) * 2.0, -10.0 + (i % 7) * 3.0
    return DetectedObject(
        object_id=f"t{i}", track_id=f"t{i}", centroid=Point2D(x=x, y=y), width=1.8, depth=1.8,
        distance=(x ** 2 + y ** 2) ** 0.5, classification=ObjectClassification.VEHICLE_LIKE, confidence=0.9,
        velocity=Velocity2D(vx=-1.0, vy=0.0), tracking_state=TrackingState.CONFIRMED,
        movement_state=MovementState.MOVING, track_age=5, track_hits=5, track_misses=0, timestamp=time.time(),
    )


def _tracked_scan(object_count: int, seq: int = 0) -> TrackedScan:
    objects = [_object(i) for i in range(object_count)]
    return TrackedScan(
        scan_id="bench", sequence_number=seq, source_id="bench", timestamp=time.time(),
        objects=objects, noise_points=[], object_count=object_count, noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _points(count: int) -> list[CartesianPoint]:
    return [CartesianPoint(angle=float(i % 360), distance=5.0, timestamp=time.time(), x=1.0, y=1.0) for i in range(count)]


def _collision_assessment(object_count: int) -> CollisionAssessment:
    results = [
        CollisionRiskResult(
            track_id=f"t{i}", classification=ObjectClassification.VEHICLE_LIKE, distance=5.0,
            relative_position=Point2D(x=5.0, y=0.0), relative_velocity=Velocity2D(vx=-1.0, vy=0.0),
            relative_speed=1.0, in_projected_path=True, ttc=3.0, collision_predicted=False,
            risk_level=RiskLevel.SAFE, risk_score=0.1, reason=["nominal"], timestamp=time.time(),
        )
        for i in range(object_count)
    ]
    return CollisionAssessment(
        scan_id="bench", sequence_number=0, source_id="bench", timestamp=time.time(),
        results=results, object_count=object_count, overall_risk=RiskLevel.SAFE,
        most_critical_object=results[0] if results else None,
    )


def _occupancy_grid(width: int = 400, height: int = 400) -> OccupancyGrid:
    cell_states = np.zeros((height, width), dtype=np.int8)
    cell_states[100:150, 100:150] = CellState.OCCUPIED
    return OccupancyGrid(
        width_cells=width, height_cells=height, resolution_m=0.1, origin_x_m=-20.0, origin_y_m=-20.0,
        max_range_m=12.0, timestamp=time.time(), scan_count=1, cell_states=cell_states, log_odds=np.zeros((height, width)),
    )


# --- Serialization + size benchmark --------------------------------------------------------------

def benchmark_serialization(settings: Settings, iterations: int = 200) -> None:
    scenarios = {
        "perception frame, no points, no objects": dict(tracked_scan=_tracked_scan(0), raw_points=None, point_mode="none"),
        "perception frame, 360 points": dict(tracked_scan=_tracked_scan(0), raw_points=_points(360), point_mode="polar"),
        "perception frame, 10 objects": dict(tracked_scan=_tracked_scan(10), raw_points=None, point_mode="none", collision_assessment=_collision_assessment(10)),
        "perception frame, 50 objects": dict(tracked_scan=_tracked_scan(50), raw_points=None, point_mode="none", collision_assessment=_collision_assessment(50)),
        "occupancy map (downsample=4)": dict(tracked_scan=_tracked_scan(0), occupancy_grid=_occupancy_grid(), include_map=True, map_downsample=4),
        "occupancy map (full resolution)": dict(tracked_scan=_tracked_scan(0), occupancy_grid=_occupancy_grid(), include_map=True, map_downsample=1),
        "maximum realistic frame (50 objects + 360 points + map)": dict(
            tracked_scan=_tracked_scan(50), raw_points=_points(360), point_mode="polar",
            collision_assessment=_collision_assessment(50), occupancy_grid=_occupancy_grid(), include_map=True, map_downsample=4,
        ),
    }

    print("Serialization time and message size per scenario:\n")
    print(f"{'scenario':<55} {'serialize avg (ms)':>20} {'size (bytes)':>14}")
    for name, kwargs in scenarios.items():
        timings = []
        size = None
        for _ in range(iterations):
            start = time.perf_counter()
            message = build_perception_frame_message(settings=settings, **kwargs)
            encoded = encode_message(message)
            timings.append((time.perf_counter() - start) * 1000.0)
            size = len(encoded)
        print(f"{name:<55} {statistics.fmean(timings):>20.4f} {size:>14,}")

    # Raw legacy protocol, separately (different wire format, not the JSON envelope above).
    raw_timings = []
    raw_size = None
    for _ in range(iterations):
        start = time.perf_counter()
        encoded = format_raw_scan(_points(360))
        raw_timings.append((time.perf_counter() - start) * 1000.0)
        raw_size = len(encoded)
    print(f"{'raw LiDAR protocol, 360 points':<55} {statistics.fmean(raw_timings):>20.4f} {raw_size:>14,}")
    print()


# --- End-to-end latency benchmark, via a real local TCP client -----------------------------------

class _BenchmarkClient:
    """A minimal, real TCP client that connects, reads on a background thread, and timestamps
    each parsed message's arrival -- standing in for `PerceptionTCPClient.cs` (see module
    docstring)."""

    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.create_connection((host, port), timeout=5)
        self.framer = MessageFramer()
        self.arrivals: list[tuple[float, dict]] = []
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        while self._running:
            try:
                self.sock.settimeout(0.5)
                chunk = self.sock.recv(1 << 20)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            now = time.time()
            for message in self.framer.feed(chunk):
                with self._lock:
                    self.arrivals.append((now, message))

    def close(self) -> None:
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass

    def perception_frame_latencies_ms(self) -> list[float]:
        with self._lock:
            arrivals = list(self.arrivals)
        latencies = []
        for arrival_time, message in arrivals:
            if message.get("message_type") != "PERCEPTION_FRAME":
                continue
            latencies.append((arrival_time - message["transmission_timestamp"]) * 1000.0)
        return latencies


def benchmark_latency_and_load(settings: Settings, rates_hz: list[float], object_counts: list[int], duration_s: float) -> None:
    print("Load test: throughput, dropped frames, and end-to-end latency at increasing rate/object-count:\n")
    print(f"{'rate (Hz)':>10} {'objects':>8} {'published':>10} {'received':>9} {'dropped':>8} {'avg lat (ms)':>13} {'p95 lat (ms)':>13} {'max lat (ms)':>13}")

    for rate_hz in rates_hz:
        for object_count in object_counts:
            local_settings = Settings(_env_file=None, streaming_json_port=_free_port(), streaming_heartbeat_interval_s=0.0)
            server = PerceptionStreamServer(settings=local_settings)
            server.start()
            client = _BenchmarkClient("127.0.0.1", local_settings.streaming_json_port)
            time.sleep(0.1)

            period_s = 1.0 / rate_hz
            published = 0
            deadline = time.perf_counter() + duration_s
            seq = 0
            while time.perf_counter() < deadline:
                loop_start = time.perf_counter()
                message = build_perception_frame_message(_tracked_scan(object_count, seq=seq), settings=local_settings)
                server.publish(message)
                published += 1
                seq += 1
                remaining = period_s - (time.perf_counter() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)

            time.sleep(0.3)  # let the sender thread + client catch up
            latencies = client.perception_frame_latencies_ms()
            received = len(latencies)
            dropped = published - received

            client.close()
            server.stop()

            avg = statistics.fmean(latencies) if latencies else 0.0
            p95 = statistics.quantiles(latencies, n=20)[18] if len(latencies) >= 20 else (max(latencies) if latencies else 0.0)
            mx = max(latencies) if latencies else 0.0
            print(f"{rate_hz:>10.0f} {object_count:>8} {published:>10} {received:>9} {dropped:>8} {avg:>13.3f} {p95:>13.3f} {mx:>13.3f}")
    print()


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main() -> None:
    setup_logging()
    parser = argparse.ArgumentParser(description="Benchmark the Phase 12 streaming layer.")
    parser.add_argument("--iterations", type=int, default=200, help="Iterations for the serialization-time benchmark.")
    parser.add_argument("--duration", type=float, default=3.0, help="Seconds per load-test cell.")
    parser.add_argument("--rates", type=float, nargs="+", default=[10.0, 20.0, 30.0])
    parser.add_argument("--objects", type=int, nargs="+", default=[1, 10, 50])
    args = parser.parse_args()

    settings = Settings(_env_file=None)
    benchmark_serialization(settings, iterations=args.iterations)
    benchmark_latency_and_load(settings, rates_hz=args.rates, object_counts=args.objects, duration_s=args.duration)


if __name__ == "__main__":
    main()
