"""A single persistent track: one `KalmanFilter2D` plus lifecycle bookkeeping (age/hits/misses,
TENTATIVE -> CONFIRMED -> COASTING -> LOST) and the most recent detection's classification/shape
data. See docs/tracking.md "Track data model" and "Track lifecycle".

`Track` itself is not a pydantic model and is never exposed outside `tracking` -- it is internal
mutable per-scan-cycle state owned by `ObjectTracker`. `to_detected_object()` is what projects it
onto the public schema tracking consumers actually see (a `DetectedObject`, per
docs/data-model.md "Extensibility rule" -- see also `models.tracking.TrackedScan`).
"""

from __future__ import annotations

import math

from common.config import Settings
from models.objects import (
    BoundingBox,
    DetectedObject,
    MovementState,
    ObjectClassification,
    Point2D,
    ShapeFeatures,
    TrackingState,
    Velocity2D,
)

from .kalman import KalmanFilter2D


class Track:
    def __init__(self, track_id: str, detection: DetectedObject, timestamp: float, settings: Settings) -> None:
        self.track_id = track_id
        self.settings = settings

        self.kf = KalmanFilter2D(
            x=detection.centroid.x,
            y=detection.centroid.y,
            process_noise_std=settings.tracking_process_noise_std_mps2,
            measurement_noise_std_m=settings.tracking_measurement_noise_std_m,
            initial_position_uncertainty_m=settings.tracking_initial_position_uncertainty_m,
            initial_velocity_uncertainty_mps=settings.tracking_initial_velocity_uncertainty_mps,
        )

        # The most recent real detection's classification/shape data -- carried forward unchanged
        # while COASTING (no fresher data exists), refreshed on every real update.
        self.classification: ObjectClassification = detection.classification
        self.classification_confidence: float = detection.confidence
        self.width: float = detection.width
        self.depth: float = detection.depth
        self.bounding_box: BoundingBox | None = detection.bounding_box
        self.point_count: int | None = detection.point_count
        self.min_distance: float | None = detection.min_distance
        self.angular_width: float | None = detection.angular_width
        self.shape_features: ShapeFeatures | None = detection.shape_features
        self.classification_reason: list[str] | None = detection.classification_reason

        self.age: int = 1
        self.hits: int = 1
        self.misses: int = 0
        # Usually TENTATIVE (1 hit is rarely enough), but a `tracking_min_hits_to_confirm` of 1
        # means this very creation already clears the bar -- checked here with the same rule
        # `update()` uses below, rather than unconditionally starting TENTATIVE and forcing a
        # redundant extra scan before a 1-hit-confirms configuration can ever report CONFIRMED.
        self.state: TrackingState = (
            TrackingState.CONFIRMED if self.hits >= settings.tracking_min_hits_to_confirm else TrackingState.TENTATIVE
        )

        # Time step (dt) handling: every `predict()` call advances the Kalman filter by one fixed
        # *nominal* scan interval (`1 / lidar_scan_frequency_hz`) rather than the measured delta
        # between consecutive `ScanFrame.timestamp` wall-clock values. This deliberately mirrors
        # `simulator.datasource.SimulatedLiDARDataSource`'s own, already-documented timing
        # principle: "obstacle motion between scans always advances by the model's nominal
        # `1 / scan_frequency_hz` ... deterministic and independent of wall-clock jitter" -- in
        # its default (non-real-time, batch/test) mode, consecutive scans are produced as fast as
        # Python can run, so wall-clock timestamp deltas are milliseconds, not the ~100ms the
        # *simulated* world actually advanced by; using them for dt was tried and measured to
        # inflate estimated velocity by ~8x (e.g. a true 2.0 m/s approach came out around 16 m/s)
        # -- see docs/tracking.md "Time step handling" for the full writeup and the numbers.
        # Reuses `lidar_scan_frequency_hz` rather than a new setting -- there is exactly one
        # configured scan rate in the system (same precedent as `preprocessing_*`'s reuse of
        # `lidar_range_min_m`/`lidar_range_max_m`, see common/config.py).
        self._nominal_dt: float = 1.0 / settings.lidar_scan_frequency_hz
        # `last_detection_timestamp`: the timestamp of the most recent *real* (matched) detection
        # -- the wall-clock input to the `tracking_track_timeout_s` check, which (unlike the
        # Kalman step above) legitimately wants real elapsed time, as a backstop against a stalled
        # data source distinct from the scan-counted `tracking_max_missed_scans` budget.
        self.last_detection_timestamp: float = timestamp

    def predict(self) -> tuple[float, float]:
        """Advance this track's Kalman filter by one nominal scan interval (`_nominal_dt` above).
        Call once per scan for every track still in the active set, before association."""
        return self.kf.predict(self._nominal_dt)

    def update(self, detection: DetectedObject, timestamp: float) -> None:
        """Apply a real, associated detection to this track: Kalman measurement-update, refresh
        classification/shape data, and advance lifecycle bookkeeping."""
        self.kf.update(detection.centroid.x, detection.centroid.y)

        self.classification = detection.classification
        self.classification_confidence = detection.confidence
        self.width = detection.width
        self.depth = detection.depth
        self.bounding_box = detection.bounding_box
        self.point_count = detection.point_count
        self.min_distance = detection.min_distance
        self.angular_width = detection.angular_width
        self.shape_features = detection.shape_features
        self.classification_reason = detection.classification_reason

        self.age += 1
        self.hits += 1
        self.misses = 0
        self.last_detection_timestamp = timestamp

        if self.state is TrackingState.TENTATIVE:
            if self.hits >= self.settings.tracking_min_hits_to_confirm:
                self.state = TrackingState.CONFIRMED
            # else: stays TENTATIVE -- not yet enough hits to graduate.
        else:
            # CONFIRMED staying CONFIRMED, or COASTING reappearing -> CONFIRMED.
            self.state = TrackingState.CONFIRMED

    def mark_missed(self, timestamp: float) -> None:
        """Record that no detection was associated to this track this scan. Call once per scan
        for every track that `predict()` was called on but that association did not match."""
        self.age += 1
        self.misses += 1

        timed_out = (
            self.misses > self.settings.tracking_max_missed_scans
            or (timestamp - self.last_detection_timestamp) > self.settings.tracking_track_timeout_s
        )
        self.state = TrackingState.LOST if timed_out else TrackingState.COASTING

    def to_detected_object(self, timestamp: float) -> DetectedObject:
        """Project this track's current internal state onto the public `DetectedObject` schema
        (see docs/tracking.md "Track data model" for the full field mapping)."""
        x, y = self.kf.x, self.kf.y
        centroid = Point2D(x=round(x, 4), y=round(y, 4))
        distance = math.hypot(x, y)

        velocity_reliable = self.hits >= self.settings.tracking_min_observations_for_velocity
        velocity = Velocity2D(vx=round(self.kf.vx, 4), vy=round(self.kf.vy, 4)) if velocity_reliable else None
        speed = velocity.speed if velocity is not None else None

        direction: float | None = None
        if velocity is not None and speed is not None and speed > 1e-6:
            direction = round(math.degrees(math.atan2(velocity.vy, velocity.vx)) % 360.0, 4)

        if not velocity_reliable:
            movement_state = MovementState.UNKNOWN
        elif speed is not None and speed < self.settings.tracking_stationary_speed_threshold_mps:
            movement_state = MovementState.STATIONARY
        else:
            movement_state = MovementState.MOVING

        predicted_x, predicted_y = self.kf.peek_predict(self._nominal_dt)

        bounding_box = None
        if self.bounding_box is not None:
            half_width = (self.bounding_box.max_x - self.bounding_box.min_x) / 2.0
            half_depth = (self.bounding_box.max_y - self.bounding_box.min_y) / 2.0
            bounding_box = BoundingBox(
                min_x=round(x - half_width, 4), max_x=round(x + half_width, 4),
                min_y=round(y - half_depth, 4), max_y=round(y + half_depth, 4),
            )

        return DetectedObject(
            object_id=self.track_id,
            track_id=self.track_id,
            centroid=centroid,
            width=self.width,
            depth=self.depth,
            distance=round(distance, 4),
            classification=self.classification,
            confidence=self.classification_confidence,
            velocity=velocity,
            direction=direction,
            bounding_box=bounding_box,
            point_count=self.point_count,
            min_distance=self.min_distance,
            angular_width=self.angular_width,
            shape_features=self.shape_features,
            classification_reason=self.classification_reason,
            predicted_position=Point2D(x=round(predicted_x, 4), y=round(predicted_y, 4)),
            tracking_state=self.state,
            movement_state=movement_state,
            track_age=self.age,
            track_hits=self.hits,
            track_misses=self.misses,
            timestamp=timestamp,
        )
