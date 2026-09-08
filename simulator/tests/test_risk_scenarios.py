"""Deterministic end-to-end scenario harness for the perception -> tracking -> clearance -> TTC
-> risk pipeline, WITHOUT physical hardware.

Each named scenario (`simulator/scenarios/<name>.json`, selectable as `--scenario <name>` in
`edge/main.py --mode simulation` / `scripts/serve_unity_bridge.py`) is driven through the EXACT
same stages `scripts/serve_unity_bridge.py` runs -- simulated LiDAR measurement -> preprocessing
-> polar->Cartesian -> DBSCAN -> classification -> tracking -> clearance + collision/TTC/risk ->
`LiveStateBuilder` -> `build_perception_frame_message` -- and this file asserts on the resulting
wire `data` dict, the exact object the dashboard renders. No value is injected: every risk / TTC
/ clearance number here is what the pipeline genuinely computes from the scenario geometry.

Scenarios cover: SAFE, CAUTION (WARNING band), LOW_CLEARANCE, CRITICAL, STATIC_OBSTACLE (TTC
N/A, not 0), APPROACHING (distance + TTC shrink), MOVING_AWAY (never CRITICAL just for
existing), MULTIPLE_OBJECTS (risk from the worst one), TRACK_LOST (normal lifecycle expiry),
FULL_360 (coordinate convention + directional sectors).
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

NAMED_SCENARIOS = [
    "safe", "caution", "low_clearance", "critical", "static_obstacle",
    "approaching", "moving_away", "multiple_objects", "track_lost", "full_360",
]

_SEVERITY = {"safe": 0, "warning": 1, "critical": 2}


def _run(scenario_id: str, scans: int, *, vehicle_speed: float = 0.0):
    """Full pipeline -> per-scan wire `data` dict, exactly as scripts/serve_unity_bridge.py builds
    it for the backend + dashboard. Returns (frames, events) where events is the flat list of
    every LiveStateEvent emitted across the run (most-recent-first within each scan)."""
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

    frames, events = [], []
    seen_event_ids: set[int] = set()
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
            for ev in live.events:
                key = (ev.sequence_number, ev.event_type, ev.track_id, ev.new_value)
                if key not in seen_event_ids:
                    seen_event_ids.add(key)
                    events.append(ev)
    return frames, events


def _confirmed_frames(frames):
    """Frames from the 3rd onward -- by then a real track has had its 3 hits to CONFIRM and its
    classification/centroid have stabilised, so risk assertions are not racing track warm-up."""
    return frames[2:]


# ============================================================================================
# Discovery -- every named scenario loads and produces a renderable, self-consistent frame
# ============================================================================================

class TestNamedScenariosAreValid:
    @pytest.mark.parametrize("scenario_id", NAMED_SCENARIOS)
    def test_scenario_runs_end_to_end(self, scenario_id):
        frames, _ = _run(scenario_id, scans=5)
        assert len(frames) == 5
        for data in frames:
            assert data["source_id"] == f"simulated:{scenario_id}"
            assert data["config"]["data_source"] == "simulation"
            assert data["risk"]["overall_risk"] in ("safe", "warning", "critical")
            assert data["clearance"] is not None
            for d in ("front", "rear", "left", "right"):
                reading = data["clearance"][d]
                assert reading["distance_m"] >= 0.0
                # The TASK 2 invariant: "no valid measurement" is NEVER zero clearance. A sector
                # with no qualifying return (nearest_point is None) must read clear-to-range > 0;
                # only a sector holding a REAL return may legitimately floor at 0 (a genuine
                # obstacle that has penetrated the safety-margin envelope).
                if reading["nearest_point"] is None:
                    assert reading["distance_m"] > 0.0
            for o in data["tracked_objects"]:
                assert o["ttc"] is None or (isinstance(o["ttc"], (int, float)) and o["ttc"] >= 0.0)


# ============================================================================================
# 1. SAFE
# ============================================================================================

class TestSafe:
    def test_far_obstacle_stays_safe_with_large_clearance_and_no_ttc(self):
        frames, _ = _run("safe", scans=8)
        for data in _confirmed_frames(frames):
            assert data["risk"]["overall_risk"] == "safe"
            assert data["clearance"]["overall_status"] == "safe"
            assert data["clearance"]["min_clearance_m"] > 1.0
            for o in data["tracked_objects"]:
                assert o["ttc"] is None            # stationary, far -> no meaningful TTC
                assert o["risk"] == "safe"


# ============================================================================================
# 2. CAUTION  (collision WARNING band -- finite TTC above the critical threshold)
# ============================================================================================

class TestCaution:
    def test_moderate_approach_is_warning_with_finite_non_critical_ttc(self):
        settings = Settings()
        frames, _ = _run("caution", scans=7)
        warned = False
        for data in _confirmed_frames(frames):
            assert data["risk"]["overall_risk"] == "warning"
            mc = data["risk"]["most_critical"]
            assert mc is not None and mc["ttc"] is not None
            # strictly inside (critical_ttc, warning_ttc] -- dangerous-but-not-imminent
            assert settings.collision_critical_ttc_s < mc["ttc"] <= settings.collision_warning_ttc_s
            warned = True
        assert warned


# ============================================================================================
# 3. LOW_CLEARANCE
# ============================================================================================

class TestLowClearance:
    def test_object_beside_corridor_is_low_clearance_but_not_a_collision(self):
        settings = Settings()
        frames, _ = _run("low_clearance", scans=6)
        for data in _confirmed_frames(frames):
            clr = data["clearance"]
            assert clr["overall_status"] == "low_clearance"
            assert clr["min_direction"] == "left"
            assert settings.clearance_critical_distance_m < clr["min_clearance_m"] <= settings.clearance_low_distance_m
            # off to the side, outside the driving corridor -> not an imminent collision
            assert data["risk"]["overall_risk"] == "safe"
            for o in data["tracked_objects"]:
                assert o["ttc"] is None


# ============================================================================================
# 4. CRITICAL
# ============================================================================================

class TestCritical:
    def test_fast_close_approach_is_critical_with_dangerous_finite_ttc(self):
        settings = Settings()
        frames, _ = _run("critical", scans=8)
        crit = [d for d in _confirmed_frames(frames) if d["risk"]["overall_risk"] == "critical"]
        assert crit, "a fast close approach must reach CRITICAL"
        for data in crit:
            mc = data["risk"]["most_critical"]
            assert mc is not None
            assert mc["ttc"] is not None and mc["ttc"] <= settings.collision_critical_ttc_s
            assert mc["distance"] > settings.min_valid_distance_m   # a real obstacle, not a self return


# ============================================================================================
# 5. STATIC_OBSTACLE  -- TTC must be N/A, never 0.0
# ============================================================================================

class TestStaticObstacle:
    def test_stationary_object_has_na_ttc_not_zero_and_a_stable_position(self):
        frames, _ = _run("static_obstacle", scans=10)
        xs = []
        for data in _confirmed_frames(frames):
            assert data["tracked_objects"], "the static pole should be tracked"
            for o in data["tracked_objects"]:
                assert o["ttc"] is None, "a non-approaching object must have TTC = N/A, not 0.0"
                if o["velocity"] is not None:
                    speed = (o["velocity"]["vx"] ** 2 + o["velocity"]["vy"] ** 2) ** 0.5
                    assert speed < 0.3      # essentially stationary
                xs.append(o["x"])
        assert max(xs) - min(xs) < 0.25, "a static obstacle's position should barely move"


# ============================================================================================
# 6. APPROACHING_OBJECT  -- distance and TTC shrink; radial velocity is closing
# ============================================================================================

class TestApproaching:
    def test_distance_and_ttc_decrease_and_risk_escalates(self):
        frames, _ = _run("approaching", scans=18)
        dists = [f["tracked_objects"][0]["distance"] for f in frames if f["tracked_objects"]]
        assert len(dists) >= 10
        assert dists[0] > dists[-1]
        # monotone non-increasing (zero sensor noise in this scenario)
        assert all(b <= a + 1e-6 for a, b in zip(dists, dists[1:]))

        finite_ttcs = [o["ttc"] for f in frames for o in f["tracked_objects"] if o["ttc"] is not None]
        assert len(finite_ttcs) >= 5
        assert finite_ttcs[0] > finite_ttcs[-1], "TTC should decrease as the object closes"

        # radial relative velocity is negative (closing) once tracking reports a velocity
        vxs = [o["velocity"]["vx"] for f in frames for o in f["tracked_objects"]
               if o["velocity"] is not None]
        assert vxs and all(vx < 0.0 for vx in vxs)

        risks = [f["risk"]["overall_risk"] for f in frames]
        assert _SEVERITY[risks[-1]] > _SEVERITY[risks[0]], f"risk should escalate: {risks}"


# ============================================================================================
# 7. MOVING_AWAY  -- never CRITICAL just because the object exists
# ============================================================================================

class TestMovingAway:
    def test_receding_object_distance_grows_ttc_na_risk_never_critical(self):
        frames, _ = _run("moving_away", scans=16)
        dists = [f["tracked_objects"][0]["distance"] for f in frames if f["tracked_objects"]]
        assert len(dists) >= 8
        assert dists[-1] > dists[0]
        assert all(b >= a - 1e-6 for a, b in zip(dists, dists[1:]))   # monotone non-decreasing

        for f in frames:
            assert f["risk"]["overall_risk"] != "critical"
            for o in f["tracked_objects"]:
                assert o["ttc"] is None      # moving away -> never a finite TTC

        vxs = [o["velocity"]["vx"] for f in frames for o in f["tracked_objects"]
               if o["velocity"] is not None]
        assert vxs and all(vx > 0.0 for vx in vxs)    # opening (receding) radial velocity


# ============================================================================================
# 8. MULTIPLE_OBJECTS  -- risk driven by the single most dangerous track
# ============================================================================================

class TestMultipleObjects:
    def test_several_tracks_risk_follows_the_worst_one(self):
        settings = Settings()
        frames, _ = _run("multiple_objects", scans=6)
        for data in _confirmed_frames(frames):
            tracks = data["tracked_objects"]
            assert len({o["track_id"] for o in tracks}) >= 3, "each obstacle is its own track"
            assert data["risk"]["overall_risk"] == "critical"
            mc = data["risk"]["most_critical"]
            assert mc is not None
            # the driver is the fast-approaching box ahead -- CRITICAL via a dangerous finite TTC,
            # not by sitting implausibly close to a centre-mounted sensor
            assert mc["in_projected_path"] is True
            assert mc["ttc"] is not None and mc["ttc"] <= settings.collision_critical_ttc_s
            assert mc["distance"] > settings.min_valid_distance_m
            # the far-off / off-to-the-side tracks each assess SAFE on their own
            others = [r for r in data["risk"]["results"] if r["track_id"] != mc["track_id"]]
            assert others and all(r["risk_level"] == "safe" for r in others)


# ============================================================================================
# 9. TRACK_LOST  -- normal lifecycle expiry, not instant deletion
# ============================================================================================

class TestTrackLost:
    def test_object_leaves_range_then_track_expires_via_lifecycle(self):
        frames, events = _run("track_lost", scans=18)

        early_ids = {o["track_id"] for f in frames[:4] for o in f["tracked_objects"]}
        assert early_ids, "the object must be tracked while it is in range"

        # it is NOT deleted the instant it stops being seen -- it coasts for several scans
        assert any(f["tracked_objects"] for f in frames[5:9]), "track should coast, not vanish instantly"

        # ...then a track_lost event fires and the roster empties out for good
        assert any(e.event_type == "track_lost" and e.track_id in early_ids for e in events)
        assert frames[-1]["tracked_objects"] == []
        assert frames[-1]["risk"]["overall_risk"] == "safe"


# ============================================================================================
# 10. FULL_360  -- coordinate convention and directional clearance sectors
# ============================================================================================

class TestFull360:
    def test_one_object_per_direction_matches_the_documented_convention(self):
        frames, _ = _run("full_360", scans=6)
        last = frames[-1]
        objs = last["tracked_objects"]
        assert len(objs) == 4, "one pole per cardinal direction"

        # +X = FRONT, -X = REAR, +Y = LEFT, -Y = RIGHT  (docs/coordinates.md)
        front = max(objs, key=lambda o: o["x"])
        rear = min(objs, key=lambda o: o["x"])
        left = max(objs, key=lambda o: o["y"])
        right = min(objs, key=lambda o: o["y"])
        assert front["x"] > 4.0 and abs(front["y"]) < 1.0
        assert rear["x"] < -4.0 and abs(rear["y"]) < 1.0
        assert left["y"] > 4.0 and abs(left["x"]) < 1.0
        assert right["y"] < -4.0 and abs(right["x"]) < 1.0

        clr = last["clearance"]
        # each sector reads a real return (not the clear-to-range default)
        for d in ("front", "rear", "left", "right"):
            assert clr[d]["nearest_point"] is not None
        # all four poles are equidistant (6 m); the reported clearances differ only by each
        # direction's own safety-envelope offset, so FRONT (largest envelope) reads smallest.
        assert clr["front"]["distance_m"] < clr["rear"]["distance_m"] < clr["left"]["distance_m"]
        assert clr["left"]["distance_m"] == pytest.approx(clr["right"]["distance_m"], abs=0.2)
        assert clr["overall_status"] == "safe"
