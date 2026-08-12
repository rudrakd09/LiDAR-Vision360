"""The versioned Python -> Unity message envelope (Phase 12).

Wraps the per-scan payload `serialization.unity_protocol.build_frame_message` (Phase 11) already
builds inside a small, explicit envelope carrying a `message_type` -- see docs/communication.md
"Protocol" for the full schema reference, an example message, and the reasoning behind every
top-level field.

Pure functions/classes only -- no socket I/O (that's `streaming.server`). Every function here is
directly unit-testable with synthetic model instances or plain dicts.
"""

from __future__ import annotations

import time
from enum import Enum

from serialization import build_frame_message

# Bumped on any breaking change to the *envelope* shape (top-level fields, not `data`'s own
# content, which `serialization.unity_protocol` documents/versions on its own additive terms).
# 2.0.0 here reflects the breaking reorganization from Phase 11's un-enveloped messages (their
# own top-level `protocol_version`/`timestamp`/`scan_id`/... are now `message_type`/`frame_id`/
# `timestamp`/`transmission_timestamp`/`data` instead) -- see docs/communication.md "Versioning
# policy". Deliberately a code constant, not a `Settings` field -- see common/config.py's own
# comment on `streaming_*` for why making this independently configurable would be actively
# wrong (it would let a misconfigured value lie about the wire format actually in use).
PROTOCOL_VERSION = "2.0.0"


class MessageType(str, Enum):
    """See docs/communication.md "Message types" for what each carries and when it's sent.

    No `RAW_LIDAR` member here, unlike some illustrative protocol sketches -- the legacy raw
    `<START>`/`<END>` line protocol (`streaming_raw_port`) is a deliberately separate, unchanged
    wire format on its own port/connection, not a message type multiplexed onto this JSON
    envelope; a client that only wants points can already get them from a `PERCEPTION_FRAME`'s
    optional `data.points` field (`streaming_point_mode`) without a redundant second
    points-only message type. See docs/communication.md "Message types" for the full reasoning.
    """

    PERCEPTION_FRAME = "PERCEPTION_FRAME"
    HEARTBEAT = "HEARTBEAT"
    SYSTEM_STATUS = "SYSTEM_STATUS"
    ERROR = "ERROR"


def _envelope(message_type: MessageType, data: dict, frame_id: int | None, timestamp: float) -> dict:
    """Build one envelope. `timestamp` is the *scan's own* timestamp (when Python captured/
    processed the data this message describes); `transmission_timestamp` is stamped here, at
    publish time -- i.e. before any per-client queueing delay. For a healthy connection (queue
    never backing up) these are effectively the same moment sending actually happens; if the
    queue *is* backed up, that delay is itself latency worth surfacing, so folding it into
    `transmission_timestamp` rather than trying to separately track "time actually on the wire"
    is a deliberate simplification -- see docs/communication.md "Timestamps" ("avoid unnecessary
    complexity if the existing system only supports one [additional] timestamp").
    """
    return {
        "protocol_version": PROTOCOL_VERSION,
        "message_type": message_type.value,
        "frame_id": frame_id,
        "timestamp": timestamp,
        "transmission_timestamp": time.time(),
        "data": data,
    }


def build_perception_frame_message(
    tracked_scan,
    collision_assessment=None,
    occupancy_grid=None,
    vehicle_state=None,
    clearance_assessment=None,
    include_map: bool = False,
    map_downsample: int = 4,
    point_mode: str = "polar",
    raw_points=None,
    settings=None,
) -> dict:
    """The main data-carrying message: one per scan. `frame_id` is the source `TrackedScan`'s own
    monotonically-increasing `sequence_number` (from `tracking.ObjectTracker`, see
    docs/tracking.md) -- not a second, independent counter; there is exactly one meaningful
    per-scan sequence number in this system, reused rather than duplicated. `point_mode` selects
    which fields `data.points` entries carry -- `"polar"` (angle/distance, the sensor's own
    native representation), `"cartesian"` (x/y), or `"both"`; `"none"` (or omitting
    `raw_points`) sends no points at all, matching Phase 11's own default (points normally travel
    over the separate legacy raw port instead). `clearance_assessment` is a `clearance.
    ClearanceEngine`-produced `ClearanceAssessment` (Phase 10); `None` (the default) means the
    caller isn't running that stage -- see `serialization.build_frame_message`'s own docstring for
    the exact None-in/None-out contract this passes through unchanged.
    """
    data = build_frame_message(
        tracked_scan,
        collision_assessment=collision_assessment,
        occupancy_grid=occupancy_grid,
        vehicle_state=vehicle_state,
        clearance_assessment=clearance_assessment,
        include_map=include_map,
        map_downsample=map_downsample,
        include_points=point_mode != "none" and raw_points is not None,
        raw_points=raw_points,
        settings=settings,
    )
    data = _apply_point_mode(data, point_mode)

    return _envelope(
        MessageType.PERCEPTION_FRAME, data,
        frame_id=tracked_scan.sequence_number, timestamp=tracked_scan.timestamp,
    )


def _apply_point_mode(data: dict, point_mode: str) -> dict:
    """Reshape `data["points"]` entries to match `point_mode` -- `unity_protocol.
    build_frame_message` always builds polar (angle/distance) entries; this adds/replaces with
    cartesian x/y where requested, computed the same way `coordinates.polar_to_cartesian` already
    does (kept as a plain, dependency-free formula here rather than importing numpy for a handful
    of points)."""
    if data.get("points") is None or point_mode not in ("cartesian", "both"):
        return data

    import math

    reshaped = []
    for point in data["points"]:
        angle_rad = math.radians(point["angle"])
        x = point["distance"] * math.cos(angle_rad)
        y = point["distance"] * math.sin(angle_rad)
        entry = {"x": round(x, 4), "y": round(y, 4)}
        if point_mode == "both":
            entry["angle"] = point["angle"]
            entry["distance"] = point["distance"]
        entry["valid"] = point["valid"]
        reshaped.append(entry)

    data["points"] = reshaped
    return data


def build_heartbeat_message(uptime_s: float, frames_sent: int, clients_connected: int) -> dict:
    """Sent on a fixed interval (`streaming_heartbeat_interval_s`) regardless of whether a real
    `PERCEPTION_FRAME` was also published in that window, so a client can distinguish "no new
    scan yet" from "the server has stopped talking to me" -- see docs/communication.md
    "Heartbeat / connection status"."""
    return _envelope(
        MessageType.HEARTBEAT,
        {"uptime_s": round(uptime_s, 3), "frames_sent": frames_sent, "clients_connected": clients_connected},
        frame_id=None, timestamp=time.time(),
    )


def build_system_status_message(status: str, source_id: str | None = None, scan_rate_hz: float | None = None) -> dict:
    """Sent once per client connection (not on a repeating interval like HEARTBEAT) -- static-ish
    session information (what scenario/source is running, the configured scan rate), for the
    HUD's own "SYSTEM: Connected" line to have something more specific to show than just
    connectivity."""
    return _envelope(
        MessageType.SYSTEM_STATUS,
        {"status": status, "source_id": source_id, "scan_rate_hz": scan_rate_hz},
        frame_id=None, timestamp=time.time(),
    )


def build_error_message(code: str, message: str) -> dict:
    """Sent when the server catches an internal error it can recover from (e.g. one scan's
    pipeline run raised, but the server itself keeps running -- see docs/communication.md
    "Non-blocking design") -- so a connected Unity client can surface *something* on the HUD
    ("server error: ...") instead of the frame stream just silently stalling with no
    explanation."""
    return _envelope(MessageType.ERROR, {"code": code, "message": message}, frame_id=None, timestamp=time.time())


class FrameIdStatus(str, Enum):
    """Result of `classify_frame_id` -- see docs/communication.md "Frame IDs" for the full
    reasoning behind treating a gap as acceptable but a duplicate/regression as not."""

    ACCEPT = "accept"          # first frame ever seen, or exactly last_frame_id + 1
    GAP = "gap"                 # newer than last seen, but one or more frame_ids were skipped -- still accept (see below)
    DUPLICATE = "duplicate"      # exactly equal to the last seen frame_id
    OUT_OF_ORDER = "out_of_order"  # older than the last seen frame_id


def classify_frame_id(last_frame_id: int | None, new_frame_id: int) -> FrameIdStatus:
    """Classify `new_frame_id` against the most recently *accepted* one.

    A **duplicate** or **out-of-order** frame should be rejected/ignored by the receiver (a
    real-time renderer must never let an older frame overwrite a newer one already displayed --
    see docs/communication.md "Frame IDs"). A **gap** (one or more frame_ids missing, but this
    one is still newer) is explicitly *not* an error and should still be accepted: real-time
    visualization needs the current state, not a guarantee that every historical frame was seen
    -- the same "latest state matters more than every historical one" principle
    `streaming.queue.LatestFrameQueue`'s drop-oldest policy already applies on the sending side.
    """
    if last_frame_id is None:
        return FrameIdStatus.ACCEPT
    if new_frame_id == last_frame_id:
        return FrameIdStatus.DUPLICATE
    if new_frame_id < last_frame_id:
        return FrameIdStatus.OUT_OF_ORDER
    if new_frame_id > last_frame_id + 1:
        return FrameIdStatus.GAP
    return FrameIdStatus.ACCEPT


def is_frame_id_acceptable(status: FrameIdStatus) -> bool:
    """Whether a receiver should actually apply/render the frame this status was computed for."""
    return status in (FrameIdStatus.ACCEPT, FrameIdStatus.GAP)
