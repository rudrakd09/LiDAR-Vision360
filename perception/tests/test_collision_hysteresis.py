"""Tests for collision.hysteresis / CollisionRiskEngine's hysteresis behavior -- the fix for a
real, reproduced bug: a static object sitting almost exactly on `collision_warning_distance_m`
(scenario `02_wall_in_front`'s wall, placed at exactly 5.0m) flip-flopped SAFE<->WARNING on nearly
every scan as ordinary sensor/preprocessing noise (sub-millimeter) crossed the boundary back and
forth. `risk.assess_risk` itself (the Phase 9 rule cascade) is completely untouched by this fix --
these tests exercise it only through `CollisionRiskEngine`, reused across multiple `evaluate()`
calls on purpose (that's what makes hysteresis state meaningful; a single one-shot call is always
identical to calling `assess_risk` directly -- see test_collision_engine.py's own existing tests,
still passing unmodified).
"""

from common.config import Settings
from collision import CollisionRiskEngine
from models.collision import RiskLevel, VehicleState
from models.objects import DetectedObject, MovementState, ObjectClassification, Point2D, TrackingState, Velocity2D
from models.tracking import TrackedScan

DEFAULT_SETTINGS = Settings(_env_file=None)


def _object(x: float, vx: float = 0.0, track_id: str = "t1") -> DetectedObject:
    """A small, stationary-or-moving object directly ahead (y=0, always in-path) -- `vx` lets a
    test drive TTC-based (not just distance-based) transitions through the exact same mechanism.

    Deliberately small (0.3m x 0.3m, at `collision_minimum_object_radius_m`'s own floor) rather
    than test_collision_engine.py's default 1.8m x 1.8m: at this file's test distances (as close
    as ~4m), a 1.8m-deep object's near edge already geometrically overlaps the vehicle's own
    front safety envelope (`vehicle_length_m/2 + front_safety_margin_m` = 3.25m by default) --
    `assess_risk`'s `collision_predicted` check (un-gated, checked first -- see risk.py) then
    correctly reports CRITICAL regardless of centroid distance, which would silently defeat these
    tests' whole point of isolating the plain distance-threshold hysteresis behavior. A small
    object keeps every distance below at a genuine, non-overlapping centroid-distance reading.
    """
    return DetectedObject(
        object_id=track_id, track_id=track_id, centroid=Point2D(x=x, y=0.0), width=0.3, depth=0.3,
        distance=abs(x), classification=ObjectClassification.VEHICLE_LIKE, confidence=0.9,
        velocity=Velocity2D(vx=vx, vy=0.0),  # always known (0.0 = confirmed stationary, not "unknown")
        tracking_state=TrackingState.CONFIRMED,
        movement_state=MovementState.MOVING if vx else MovementState.STATIONARY,
        track_age=5, track_hits=5, track_misses=0, timestamp=0.0,
    )


def _scan(obj: DetectedObject, sequence_number: int = 0) -> TrackedScan:
    return TrackedScan(
        scan_id="s", sequence_number=sequence_number, source_id="unit-test", timestamp=1000.0 + sequence_number,
        objects=[obj], noise_points=[], object_count=1, noise_count=0,
        new_track_count=0, lost_track_count=0, coasting_track_count=0,
    )


def _engine() -> CollisionRiskEngine:
    return CollisionRiskEngine(settings=DEFAULT_SETTINGS)


def _risk_at(engine: CollisionRiskEngine, x: float, seq: int, vx: float = 0.0, track_id: str = "t1") -> RiskLevel:
    return engine.evaluate(_scan(_object(x, vx=vx, track_id=track_id), sequence_number=seq)).results[0].risk_level


WARN_DIST = DEFAULT_SETTINGS.collision_warning_distance_m  # 5.0
CRIT_DIST = DEFAULT_SETTINGS.collision_critical_distance_m  # 2.0
DIST_MARGIN = DEFAULT_SETTINGS.collision_risk_hysteresis_distance_margin_m  # 0.5


class TestExactlyAtThresholdJitter:
    """The literal repro: a static object whose measured distance jitters by ~1mm around
    `collision_warning_distance_m`, exactly like scenario 02_wall_in_front's real-world behavior
    (verified live: dozens of SAFE<->WARNING transitions per second before this fix)."""

    def test_millimeter_jitter_around_warning_boundary_does_not_flip_back_to_safe(self):
        engine = _engine()
        # Establish WARNING first (clearly inside the threshold).
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING

        # Now jitter across the exact boundary by ~1mm, alternating, many scans -- like the real
        # capture (frame-to-frame distance 4.9998, 5.0006, 5.0003, 4.9995, ...).
        jittered_distances = [5.0006, 5.0003, 4.9998, 4.9995, 5.0004, 5.0009, 4.9998, 5.0000, 4.9992]
        for i, d in enumerate(jittered_distances, start=1):
            level = _risk_at(engine, d, seq=i)
            assert level == RiskLevel.WARNING, f"scan {i} (distance={d}) flipped to {level.value} -- hysteresis failed to hold"

    def test_without_ever_recovering_the_object_stays_warning_indefinitely(self):
        engine = _engine()
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        # 30 scans of jitter strictly within the recovery margin (5.0 to 5.0 + 0.5 = 5.5).
        for i in range(1, 31):
            d = WARN_DIST + (0.001 if i % 2 == 0 else -0.001)
            assert _risk_at(engine, d, seq=i) == RiskLevel.WARNING


class TestGenuineThresholdCrossing:
    """Escalation (risk getting worse) must be immediate -- no multi-scan delay, ever."""

    def test_safe_to_warning_is_immediate_on_the_very_next_scan(self):
        engine = _engine()
        assert _risk_at(engine, 10.0, seq=0) == RiskLevel.SAFE
        # A real approach: distance drops decisively inside the warning threshold.
        assert _risk_at(engine, WARN_DIST - 1.0, seq=1) == RiskLevel.WARNING

    def test_warning_to_critical_is_also_immediate(self):
        engine = _engine()
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        assert _risk_at(engine, CRIT_DIST - 0.5, seq=1) == RiskLevel.CRITICAL

    def test_first_ever_assessment_of_a_track_matches_plain_assess_risk_unfiltered(self):
        """A track's very first scan has no history to hold against -- must equal the raw,
        unmodified Phase 9 result exactly (proves hysteresis is invisible on first contact)."""
        from collision.risk import assess_risk

        engine = _engine()
        result = engine.evaluate(_scan(_object(WARN_DIST, vx=0.0))).results[0]
        raw_level, _ = assess_risk(True, WARN_DIST, None, 0.0, False, None, DEFAULT_SETTINGS)
        assert result.risk_level == raw_level == RiskLevel.WARNING


class TestRecovery:
    """De-escalation (risk improving) must only be accepted once the real metric clears the
    threshold by the configured margin -- not merely crosses the bare threshold."""

    def test_barely_past_the_bare_threshold_does_not_yet_recover(self):
        engine = _engine()
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        # Past the bare 5.0m threshold, but not past 5.0 + 0.5 = 5.5m recovery margin.
        assert _risk_at(engine, WARN_DIST + 0.2, seq=1) == RiskLevel.WARNING
        assert _risk_at(engine, WARN_DIST + (DIST_MARGIN - 0.01), seq=2) == RiskLevel.WARNING

    def test_clearing_the_recovery_margin_returns_to_safe(self):
        engine = _engine()
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        assert _risk_at(engine, WARN_DIST + 0.2, seq=1) == RiskLevel.WARNING  # not recovered yet
        assert _risk_at(engine, WARN_DIST + DIST_MARGIN + 0.1, seq=2) == RiskLevel.SAFE  # now recovered

    def test_recovery_is_evaluated_against_the_real_distance_not_frozen_forever(self):
        """Hysteresis delays recovery -- it must never make it permanent: an object that keeps
        receding well past the margin recovers, and one that then re-approaches escalates again
        immediately (both directions keep working after a hysteresis hold)."""
        engine = _engine()
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        assert _risk_at(engine, WARN_DIST + DIST_MARGIN + 1.0, seq=1) == RiskLevel.SAFE
        assert _risk_at(engine, WARN_DIST - 1.0, seq=2) == RiskLevel.WARNING  # re-approach -- immediate again


class TestCriticalBehaviour:
    """CRITICAL must still escalate immediately (never delayed/masked by hysteresis), and its own
    recovery is subject to the same margin -- it must not relax straight past WARNING's own
    recovery check just because the raw reading briefly dipped to WARNING."""

    def test_critical_escalation_is_immediate_even_from_safe(self):
        engine = _engine()
        assert _risk_at(engine, 20.0, seq=0) == RiskLevel.SAFE
        assert _risk_at(engine, CRIT_DIST - 0.5, seq=1) == RiskLevel.CRITICAL

    def test_critical_via_ttc_is_also_immediate(self):
        engine = _engine()
        assert _risk_at(engine, 20.0, seq=0) == RiskLevel.SAFE
        # Fast closing speed at moderate distance -- triggers via TTC <= collision_critical_ttc_s,
        # not the distance rule (proves hysteresis applies uniformly, not just to the distance path).
        level = _risk_at(engine, 8.0, seq=1, vx=-20.0)
        assert level == RiskLevel.CRITICAL

    def test_critical_does_not_relax_to_warning_within_its_own_recovery_margin(self):
        engine = _engine()
        assert _risk_at(engine, CRIT_DIST - 0.5, seq=0) == RiskLevel.CRITICAL
        # Raw distance now reads WARNING (2.0 < 2.3 <= 5.0), but still within CRITICAL's own
        # recovery margin (critical_distance + margin = 2.0 + 0.5 = 2.5) -- must hold at CRITICAL.
        assert _risk_at(engine, CRIT_DIST + 0.3, seq=1) == RiskLevel.CRITICAL

    def test_critical_relaxes_to_warning_once_past_its_recovery_margin(self):
        engine = _engine()
        assert _risk_at(engine, CRIT_DIST - 0.5, seq=0) == RiskLevel.CRITICAL
        # Comfortably past both the critical recovery margin (2.5m) AND clear of the vehicle's own
        # static front envelope (~3.4m at this object's size) -- see _object's own docstring on
        # why a distance just barely past the critical margin (e.g. 2.6m) still overlaps the
        # envelope and would correctly stay CRITICAL regardless of hysteresis.
        assert _risk_at(engine, WARN_DIST - 1.0, seq=1) == RiskLevel.WARNING

    def test_critical_millimeter_jitter_at_its_own_boundary_does_not_flip_to_warning(self):
        engine = _engine()
        assert _risk_at(engine, CRIT_DIST - 0.5, seq=0) == RiskLevel.CRITICAL
        for i, d in enumerate([CRIT_DIST + 0.001, CRIT_DIST - 0.0005, CRIT_DIST + 0.0009, CRIT_DIST - 0.0002], start=1):
            level = _risk_at(engine, d, seq=i)
            assert level == RiskLevel.CRITICAL, f"scan {i} (distance={d}) flipped to {level.value}"


class TestConfigurability:
    """Requirement: hysteresis margins must be configurable, not hard-coded."""

    def test_zero_margin_reproduces_the_original_unfiltered_flip_flopping_behavior(self):
        settings = Settings(_env_file=None, collision_risk_hysteresis_distance_margin_m=0.0, collision_risk_hysteresis_ttc_margin_s=0.0)
        engine = CollisionRiskEngine(settings=settings)
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        # With zero margin, a genuine (if tiny) crossing past the bare threshold recovers immediately.
        assert _risk_at(engine, WARN_DIST + 0.001, seq=1) == RiskLevel.SAFE

    def test_larger_margin_holds_longer(self):
        settings = Settings(_env_file=None, collision_risk_hysteresis_distance_margin_m=2.0, collision_risk_hysteresis_ttc_margin_s=0.0)
        engine = CollisionRiskEngine(settings=settings)
        assert _risk_at(engine, WARN_DIST - 0.5, seq=0) == RiskLevel.WARNING
        assert _risk_at(engine, WARN_DIST + 1.5, seq=1) == RiskLevel.WARNING  # still within the wider 2.0m margin
        assert _risk_at(engine, WARN_DIST + 2.5, seq=2) == RiskLevel.SAFE


class TestPhase9AlgorithmUnchanged:
    """`risk.assess_risk` itself must remain exactly as it was -- these tests call it directly,
    with no engine/hysteresis involved at all, matching test_collision_risk.py's own existing
    coverage style."""

    def test_assess_risk_still_pure_and_stateless(self):
        from collision.risk import assess_risk

        first = assess_risk(True, WARN_DIST + 0.1, None, 0.0, False, None, DEFAULT_SETTINGS)
        second = assess_risk(True, WARN_DIST + 0.1, None, 0.0, False, None, DEFAULT_SETTINGS)
        assert first[0] == second[0] == RiskLevel.SAFE  # identical inputs -> identical output, no memory

    def test_assess_risk_thresholds_are_bare_no_margin_applied(self):
        from collision.risk import assess_risk

        level, _ = assess_risk(True, WARN_DIST, None, 0.0, False, None, DEFAULT_SETTINGS)
        assert level == RiskLevel.WARNING  # exactly on the plain threshold -- unchanged Phase 9 behavior
