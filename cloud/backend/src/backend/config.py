"""Backend configuration -- re-exports `perception`'s own `Settings`/`get_settings` rather than
building a parallel config system.

Every value this service needs (`backend_host`, `backend_port`, `backend_cors_origins`,
`backend_ring_buffer_size`, `backend_event_default_limit`/`backend_event_max_limit`,
`backend_track_grace_period_s`, `database_url`, `backend_sqlite_fallback_path`,
`streaming_host`/`streaming_json_port` -- where to connect to ingest from) already lives on
`perception.common.config.Settings`, added there following that file's own established rule
("nothing in this project should hard-code... every such value lives on Settings") -- see that
module for the full documented defaults/reasoning behind each one. Duplicating a second `Settings`
class here would mean two independently-configurable notions of e.g. `streaming_json_port`, which
could disagree; there is exactly one.
"""

from __future__ import annotations

from common.config import Settings, get_settings

__all__ = ["Settings", "get_settings"]
