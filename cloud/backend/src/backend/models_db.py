"""SQLAlchemy ORM models -- session records and risk/clearance *transition* events only.

**Deliberately not one row per raw frame.** Raw `PERCEPTION_FRAME`s arrive continuously (10-30Hz)
and would grow the database unbounded for no real benefit -- "recent state" is already served from
`state.LatestState`'s bounded in-memory ring buffer (`Settings.backend_ring_buffer_size`). What
*is* worth persisting is discrete, meaningful moments: a session starting/ending, and each time the
collision or clearance state actually changes -- see docs/cloud.md "Database".
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
    """One row per bridge connection this backend has ingested from (identified by the
    `SYSTEM_STATUS` message's `source_id`, i.e. the scenario) -- opened when ingestion first
    connects or receives a new `SYSTEM_STATUS`, closed (`ended_at` set) on disconnect."""

    __tablename__ = "sessions"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    source_id: Mapped[str | None] = mapped_column(String, nullable=True)
    scan_rate_hz: Mapped[float | None] = mapped_column(Float, nullable=True)
    started_at: Mapped[float] = mapped_column(Float, default=time.time)
    ended_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    frame_count: Mapped[int] = mapped_column(Integer, default=0)
    status: Mapped[str] = mapped_column(String, default="active")  # "active" | "ended"


class CollisionEvent(Base):
    """One row per collision `risk_level` *transition* for the vehicle-level `overall_risk`
    (recorded only when it changes from the previous frame, not every frame) -- see
    `ingestion.PerceptionIngestor._detect_transitions`."""

    __tablename__ = "collision_events"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    frame_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float)
    recorded_at: Mapped[float] = mapped_column(Float, default=time.time)

    risk_level: Mapped[str] = mapped_column(String)
    previous_risk_level: Mapped[str | None] = mapped_column(String, nullable=True)
    track_id: Mapped[str | None] = mapped_column(String, nullable=True)
    classification: Mapped[str | None] = mapped_column(String, nullable=True)
    distance_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    ttc_s: Mapped[float | None] = mapped_column(Float, nullable=True)
    reason: Mapped[str | None] = mapped_column(String, nullable=True)  # joined reason bullets


class ClearanceEvent(Base):
    """One row per clearance `overall_status` *transition* -- same "only on change" policy as
    `CollisionEvent`."""

    __tablename__ = "clearance_events"

    id: Mapped[str] = mapped_column(String, primary_key=True, default=_uuid)
    session_id: Mapped[str] = mapped_column(String, index=True)
    frame_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    timestamp: Mapped[float] = mapped_column(Float)
    recorded_at: Mapped[float] = mapped_column(Float, default=time.time)

    overall_status: Mapped[str] = mapped_column(String)
    previous_status: Mapped[str | None] = mapped_column(String, nullable=True)
    min_direction: Mapped[str | None] = mapped_column(String, nullable=True)
    min_clearance_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    corridor_width_m: Mapped[float | None] = mapped_column(Float, nullable=True)
    reason: Mapped[str | None] = mapped_column(String, nullable=True)
