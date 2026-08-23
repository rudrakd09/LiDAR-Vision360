"""SQLAlchemy ORM models -- PostgreSQL (or the local SQLite fallback, same schema) is this
project's PERSISTENCE/HISTORY layer only; it is never the live transport (that is `/ws/live`,
served from `state.LatestState`'s in-memory ring buffer -- see docs/architecture.md "PostgreSQL as
history, WebSocket as live transport"). Every table here is a discrete, meaningful moment (a
session starting/ending, a track appearing/disappearing, a risk/clearance/TTC/sensor transition),
sourced directly from the Edge's own `LiveState`/`LiveState.events` -- never re-derived or
invented downstream (see `ingestion.PerceptionIngestor._persist_events`).

**Deliberately not one row per raw frame.** Raw `PERCEPTION_FRAME`s (and their 360-point scans)
arrive continuously (10-30Hz) and would grow the database unbounded for no real benefit -- "recent
state" is already served from `state.LatestState`'s bounded in-memory ring buffer (`Settings.
backend_ring_buffer_size`). The one exception is `TrackRecord`: it *is* updated on every frame a
track appears in, but that is an UPDATE to a bounded set of rows (one per unique `track_id` ever
seen this session), not an unbounded INSERT stream -- see its own docstring.

Every table below carries `session_id`/`source_id`/`timestamp` (this project's own requirement for
"every persisted record must be traceable to which session, which source, and when").
"""

from __future__ import annotations

import time
import uuid

from sqlalchemy import Float, Integer, String
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


class Base(DeclarativeBase):
    pass


def _uuid() -> str:
    return str(uuid.uuid4())


class SessionRecord(Base):
    """One row per bridge connection this backend has ingested from -- opened when ingestion
    first connects, closed (`ended_at` set) on disconnect. `id` is this table's own synthetic
    primary key (used as the `session_id` foreign-key value on every event table below, and as
    the `?session_id=` filter `routes/events.py` already accepts) -- distinct from
    `edge_session_id`, the Edge-minted wire identity (`pipeline.LiveStateBuilder.session_id`, see
    docs/architecture.md "Session and sequence management"). Both are kept: `id` is a stable
    DB-native key even across a hypothetical future re-ingestion of the same Edge session;
    `edge_session_id` is what actually proves "is this the same real perception session" and is
    what `state.ConnectionInfo.session_id`/`GET /debug/stream-status` report as `session_id`.
    """

    __tablename__ = "sessions"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    edge_session_id: Mapped[str | None] = mapped_column(String, nullable=True, index=True)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    scan_rate_hz: Mapped[float | None] = mapped_column(Float, nullable=True)
    started_at: Mapped[float] = mapped_column(Float, default=time.time)
    ended_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    frame_count: Mapped[int] = mapped_column(Integer, default=0)
    status: Mapped[str] = mapped_column(String, default="active")  # "active" | "ended"


class TrackRecord(Base):
    """One row per unique `track_id` ever seen in a session -- created the scan a `track_created`
    LiveState event arrives, updated (last_seen_at/frames_tracked/classification) on every
    subsequent scan that same track_id appears in `PerceptionFrameData.tracked_objects`, and
    marked `status="lost"` the scan a `track_lost` event arrives. This is what answers "what
    tracks existed in session X, what were they, how long were they tracked" from history after
    the live session has ended -- `state.LatestState.tracks_roster()` only ever answers that for
    the CURRENT session, in memory. Bounded: one row per track_id, updated in place, never one row
    per frame."""

    __tablename__ = "tracks"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    track_id: Mapped[str] = mapped_column(String, index=True)
    classification: Mapped[str | None] = mapped_column(String, nullable=True)
    sensor_source: Mapped[str | None] = mapped_column(String, nullable=True)
    first_seen_at: Mapped[float] = mapped_column(Float)
    last_seen_at: Mapped[float] = mapped_column(Float)
    frames_tracked: Mapped[int | None] = mapped_column(Integer, nullable=True)
    status: Mapped[str] = mapped_column(String, default="active")  # "active" | "lost"


class CollisionEvent(Base):
    """One row per collision `risk_level` *transition* for the vehicle-level `overall_risk`
    (recorded only when it changes from the previous frame, not every frame) -- sourced directly
    from the Edge's own `LiveState.events` (`event_type == "collision"`), enriched with the
    matching `risk.most_critical` detail from the same frame -- see
    `ingestion.PerceptionIngestor._persist_events`. `collision_predicted` is the supporting
    boolean `collision.CollisionRiskEngine`'s own footprint-intersection simulation produced for
    the most-critical object -- this project's pipeline has exactly one real collision/risk
    concept (SAFE/WARNING/CRITICAL, with this boolean alongside it), not two independently
    invented ones, so it is a column here rather than a second "collision events" table."""

    __tablename__ = "collision_events"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    frame_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float)
    recorded_at: Mapped[float] = mapped_column(Float, default=time.time)

    risk_level: Mapped[str] = mapped_column(String)
    previous_risk_level: Mapped[str | None] = mapped_column(String, nullable=True)
    track_id: Mapped[str | None] = mapped_column(String, nullable=True)
    classification: Mapped[str | None] = mapped_column(String, nullable=True)
    distance_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    ttc_s: Mapped[float | None] = mapped_column(Float, nullable=True)
    collision_predicted: Mapped[bool | None] = mapped_column(nullable=True)
    reason: Mapped[str | None] = mapped_column(String, nullable=True)  # joined reason bullets


class ClearanceEvent(Base):
    """One row per clearance `overall_status` *transition* -- same "only on change" policy as
    `CollisionEvent`, same Edge-sourced-not-re-derived origin (`event_type == "clearance"`)."""

    __tablename__ = "clearance_events"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    frame_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float)
    recorded_at: Mapped[float] = mapped_column(Float, default=time.time)

    overall_status: Mapped[str] = mapped_column(String)
    previous_status: Mapped[str | None] = mapped_column(String, nullable=True)
    min_direction: Mapped[str | None] = mapped_column(String, nullable=True)
    min_clearance_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    corridor_width_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    reason: Mapped[str | None] = mapped_column(String, nullable=True)


class TTCEvent(Base):
    """One row per track whose TTC transitioned between defined/undefined (started or stopped
    "approaching" -- see `models.collision.CollisionRiskResult.ttc`'s own None-means-not-
    approaching contract) -- sourced from the Edge's own `LiveState.events`
    (`event_type == "ttc_change"`), enriched with the matching `risk.results[]` entry for this
    track_id from the same frame. Distinct from `CollisionEvent`: a TTC transition can happen
    without the vehicle-level `overall_risk` changing (a different, less-critical track started
    approaching while a worse one already held CRITICAL), so this is real, additional signal, not
    a duplicate of the collision event."""

    __tablename__ = "ttc_events"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    frame_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float)
    recorded_at: Mapped[float] = mapped_column(Float, default=time.time)

    track_id: Mapped[str | None] = mapped_column(String, nullable=True)
    previous_value: Mapped[str | None] = mapped_column(String, nullable=True)  # "approaching" | "not_approaching"
    new_value: Mapped[str] = mapped_column(String)
    ttc_s: Mapped[float | None] = mapped_column(Float, nullable=True)
    classification: Mapped[str | None] = mapped_column(String, nullable=True)
    distance_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    reason: Mapped[str | None] = mapped_column(String, nullable=True)


class SensorEvent(Base):
    """One row per real LiDAR sensor status transition -- currently: data-quality crossing
    `Settings.sensor_quality_degraded_threshold_percent` (`PerceptionFrameData.sensor_status.
    lidar.valid_percentage`), detected at the Edge (`pipeline.LiveStateBuilder`). Radar/STM32
    never produce events in this project's current scope -- they are always absent
    (`sensor_status.radar` is always `null`; see README "Important sensor limitation"), and an
    always-absent value never transitions. This table exists so a genuinely degraded scan (heavy
    noise/outliers, e.g. scenario `09_noisy_lidar`/`10_missing_outliers`) leaves a real, queryable
    history entry rather than only a transient dashboard indicator."""

    __tablename__ = "sensor_events"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    frame_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float)
    recorded_at: Mapped[float] = mapped_column(Float, default=time.time)

    sensor_type: Mapped[str] = mapped_column(String, default="lidar")
    previous_status: Mapped[str | None] = mapped_column(String, nullable=True)
    new_status: Mapped[str] = mapped_column(String)
    valid_percentage: Mapped[float | None] = mapped_column(Float, nullable=True)
    summary: Mapped[str | None] = mapped_column(String, nullable=True)
