"""Real-time Python <-> Unity communication layer (Phase 12): a versioned, structured JSON
protocol (`PerceptionStreamServer`) alongside the legacy raw `<START>`/`<END>` line protocol
(`RawLidarStreamServer`, unchanged wire format) -- see docs/communication.md.

Status: **implemented**. Independent of `simulator`; depends only on `perception`'s own
`serialization` package (Phase 11) and `common.config.Settings` for its own `streaming_*`
parameters -- see docs/architecture.md.

Typical usage:

    from streaming import PerceptionStreamServer, RawLidarStreamServer
    from streaming.protocol import build_perception_frame_message

    json_server = PerceptionStreamServer()   # reads defaults from common.config.Settings
    raw_server = RawLidarStreamServer()
    json_server.start()
    raw_server.start()

    for tracked_scan in ...:
        message = build_perception_frame_message(tracked_scan, ...)
        json_server.publish(message)          # never blocks
        raw_server.publish_scan(cartesian_scan.points)

    json_server.stop()
    raw_server.stop()
"""

from .framing import MessageFramer, encode_message
from .latest_frame_queue import LatestFrameQueue
from .protocol import (
    PROTOCOL_VERSION,
    FrameIdStatus,
    MessageType,
    build_error_message,
    build_heartbeat_message,
    build_perception_frame_message,
    build_system_status_message,
    classify_frame_id,
    is_frame_id_acceptable,
)
from .server import PerceptionStreamServer, RawLidarStreamServer, format_raw_scan

__all__ = [
    "PerceptionStreamServer",
    "RawLidarStreamServer",
    "format_raw_scan",
    "LatestFrameQueue",
    "MessageFramer",
    "encode_message",
    "PROTOCOL_VERSION",
    "MessageType",
    "FrameIdStatus",
    "build_perception_frame_message",
    "build_heartbeat_message",
    "build_system_status_message",
    "build_error_message",
    "classify_frame_id",
    "is_frame_id_acceptable",
]
