"""Phase 4 -- verify that the wire payload the Dashboard renders reflects the current scenario.

Runs each simulation scenario through the FULL perception pipeline + `pipeline.LiveStateBuilder`
+ `streaming.protocol.build_perception_frame_message`, then asserts the resulting `data` dict
(the exact object the dashboard's `PerceptionFrameData` type mirrors and every panel reads
verbatim -- see docs/architecture.md "Dashboard and Unity as pure LiveState consumers"). No
browser needed: the dashboard renders these fields as-is, so asserting them here is a direct
proxy for "the dashboard shows X for scenario Y".

Covers §17 (all 10 scenarios) and the §22 manual acceptance checks (01/04/05/08).
"""

from __future__ import annotations

import pytest

from clearance import ClearanceEngine
from clustering import DBSCANClusterer
from collision import CollisionRiskEngine
from common.config import Settings
from coordinates import CoordinateTransformer
from fusion import FusionEngine
from models.collision import VehicleState
from objects import GeometricClassifier
from pipeline import LiveStateBuilder
from preprocessing import Preprocessor
from simulator.scenarios import make_data_source
from streaming.protocol import build_perception_frame_message
from tracking import ObjectTracker

ALL_SCENARIOS = [
    "01_empty", "02_wall_in_front", "03_pole_left", "04_vehicle_ahead", "05_multiple_obstacles",
    "06_narrow_corridor", "07_moving_crossing", "08_approaching_obstacle", "09_noisy_lidar",
    "10_missing_outliers",
]


def _run_wire_frames(scenario_id: str, scans: int, *, vehicle_speed: float = 0.0):
    """Full pipeline -> LiveState -> wire `data` dicts, exactly as `scripts/serve_unity_bridge.py`
    builds them for the backend + Unity + (via /ws/live) the dashboard."""
    # Constructed exactly as scripts/serve_unity_bridge.py does (most stages read get_settings()
    # themselves; only fusion + LiveStateBuilder take an explicit settings arg there).
    settings = Settings()
    pre = Preprocessor()
    tf = CoordinateTransformer()
    cl = DBSCANClusterer()
    gc = GeometricClassifier()
    tr = ObjectTracker()
    fe = FusionEngine(settings=settings)
    ce = CollisionRiskEngine()
    cle = ClearanceEngine()
    lsb = LiveStateBuilder(settings=settings)
    vs = VehicleState(speed_mps=vehicle_speed)
    src = make_data_source(scenario_id, settings=settings)

    frames = []
    with src:
        for _ in range(scans):
            raw = src.read_scan()
            clean = pre.process(raw)
            cart = tf.transform(clean)
            clustered = cl.cluster(cart)
            classified = gc.classify(clustered)
            tracked = tr.update(classified)
            tracked = fe.fuse(tracked, getattr(src, "latest_radar_reading", None))
            assessment = ce.evaluate(tracked, vehicle_state=vs)
            clearance = cle.evaluate(cart, vehicle_state=vs)
            live = lsb.build(tracked_scan=tracked, preprocessed_scan=clean,
                             collision_assessment=assessment, clearance_assessment=clearance)
            msg = build_perception_frame_message(
                tracked, collision_assessment=assessment, vehicle_state=vs,
                clearance_assessment=clearance, settings=settings,
                session_id=live.session_id, live_state=live,
            )
            frames.append(msg["data"])
    return frames


# ============================================================================================
# §17 -- every scenario produces a well-formed, self-consistent wire frame
# ============================================================================================

class TestAllScenariosProduceRenderableFrames:
    @pytest.mark.parametrize("scenario_id", ALL_SCENARIOS)
    def test_frame_shape_and_source(self, scenario_id):
        for data in _run_wire_frames(scenario_id, scans=5):
            assert data["source_id"] == f"simulated:{scenario_id}"
            assert data["config"]["data_source"] == "simulation"
            assert isinstance(data["sequence_number"], int)
            assert isinstance(data["objects"], list)
            assert data["tracked_objects"] is not None
            # object count the dashboard shows == len(tracked_objects); never fabricated
            assert len(data["tracked_objects"]) == sum(1 for o in data["objects"] if o["track_id"])
            assert data["risk"] is not None
            assert data["risk"]["overall_risk"] in ("safe", "warning", "critical")
            assert data["clearance"] is not None
            for d in ("front", "rear", "left", "right"):
                assert data["clearance"][d]["distance_m"] >= 0.0
            # every tracked object's TTC is either a real number or explicitly null (-> "N/A")
            for t in data["tracked_objects"]:
                assert t["ttc"] is None or (isinstance(t["ttc"], (int, float)) and t["ttc"] >= 0.0)
                assert t["risk"] in ("safe", "warning", "critical", None)
                assert t["classification"] in (
                    "wall", "vehicle_like", "pole_like", "person_like", "large_obstacle", "unknown",
                )


# ============================================================================================
# §22 -- manual acceptance checks, as wire-frame assertions
# ============================================================================================

class TestManualAcceptanceScenarios:
    def test_01_empty_objects_tracks_zero_risk_safe(self):
        for data in _run_wire_frames("01_empty", scans=5):
            assert data["objects"] == []
            assert data["tracked_objects"] == []
            assert data["risk"]["overall_risk"] == "safe"
            assert data["risk"]["most_critical"] is None

    def test_04_vehicle_ahead_object_track_distance_risk_visible(self):
        frames = _run_wire_frames("04_vehicle_ahead", scans=8)
        last = frames[-1]
        assert len(last["tracked_objects"]) >= 1
        obj = last["tracked_objects"][0]
        assert obj["track_id"]                      # a track id exists
        assert obj["classification"]               # an object type exists
        assert obj["distance"] > 0.0               # a distance exists
        assert obj["risk"] in ("safe", "warning", "critical")   # risk visible
        # TTC is shown when valid (stationary vehicle + stationary ego -> may legitimately be null)
        assert obj["ttc"] is None or obj["ttc"] >= 0.0

    def test_05_multiple_obstacles_multiple_tracks_distinct_ids(self):
        frames = _run_wire_frames("05_multiple_obstacles", scans=8)
        last = frames[-1]
        ids = [o["track_id"] for o in last["tracked_objects"]]
        assert len(ids) >= 2
        assert len(set(ids)) == len(ids)          # distinct track ids

    def test_07_moving_crossing_track_persists_as_object_moves(self):
        frames = _run_wire_frames("07_moving_crossing", scans=12)
        # a track id that appears in an early frame and is still present several frames later,
        # with its position having changed
        early = {o["track_id"]: (o["x"], o["y"]) for o in frames[3]["tracked_objects"]}
        late = {o["track_id"]: (o["x"], o["y"]) for o in frames[-1]["tracked_objects"]}
        persistent = set(early) & set(late)
        assert persistent, "expected at least one track to persist across the crossing"
        moved = any(early[t] != late[t] for t in persistent)
        assert moved, "a persistent track's position should change as the object crosses"

    def test_08_approaching_obstacle_distance_ttc_risk_change_over_time(self):
        frames = _run_wire_frames("08_approaching_obstacle", scans=20, vehicle_speed=0.0)
        # distance to the (single) approaching object decreases over the run
        def dist(f):
            objs = f["tracked_objects"]
            return objs[0]["distance"] if objs else None
        dists = [d for d in (dist(f) for f in frames) if d is not None]
        assert len(dists) >= 5
        assert dists[0] > dists[-1], f"distance should shrink: {dists[0]:.2f} -> {dists[-1]:.2f}"
        # risk escalates at some point (SAFE early, more severe later)
        risks = [f["risk"]["overall_risk"] for f in frames]
        severity = {"safe": 0, "warning": 1, "critical": 2}
        assert max(severity[r] for r in risks) > severity[risks[0]], f"risk should escalate: {risks}"
        # a finite TTC appears once the object is genuinely closing
        ttcs = [o["ttc"] for f in frames for o in f["tracked_objects"] if o["ttc"] is not None]
        assert ttcs, "a finite TTC should appear as the obstacle approaches"
