# Real-Time Python ↔ Unity Communication

## Status

**Implemented (Phase 12)**, in `perception/src/streaming/`. Builds on Phase 11 (Unity Digital
Twin) -- this phase supersedes Phase 11's own first-cut streaming code
(`scripts/serve_unity_bridge.py`'s inline `ClientBroadcaster`, and the un-enveloped JSON messages
`perception/src/serialization/unity_protocol.py` used to send directly) with a proper,
non-blocking, versioned, envelope-based protocol. See "Versioning policy" below for exactly what
changed and why.

**Important environment caveat, carried over from Phase 11**: the Python side of this phase is
fully implemented and tested against real TCP sockets (see "Testing"). The Unity C# side was
written and cross-checked carefully but, as with Phase 11, **could not be compiled or run in a
live Unity Editor** (none was available in the build environment). Treat the Unity-side changes
as thoroughly-reviewed source, not verified-working software, until opened in the Editor -- see
docs/unity.md "Status"/"Troubleshooting".

## Objective

A robust, real-time, non-blocking communication layer between the Python perception engine and
the Unity digital twin, transmitting raw/Cartesian points, classified objects, persistent track
IDs, positions/velocities/predictions, occupancy map data, collision risk, TTC, and system status
-- at 10Hz minimum, substantially higher where practical (see "Performance").

## Architecture principle

Python remains the sole authority for perception (clustering, classification, tracking, Kalman
filtering, mapping, collision/TTC). Unity only renders what Python already computed and sent --
verified by construction, same as Phase 11: no file under `unity/LiDARVision360/Assets/Scripts/`
contains a clustering loop, a filter/prediction step, or a risk-threshold comparison against a
Unity-side constant.

## Protocol

### Message envelope

Every message on the structured JSON port shares one envelope shape
(`perception/src/streaming/protocol.py`):

```jsonc
{
  "protocol_version": "2.0.0",
  "message_type": "PERCEPTION_FRAME",   // | "HEARTBEAT" | "SYSTEM_STATUS" | "ERROR"
  "frame_id": 42,                        // monotonically increasing; null for non-PERCEPTION_FRAME types
  "timestamp": 1234567890.123,           // the scan's own timestamp (when Python captured/processed it)
  "transmission_timestamp": 1234567890.130,  // when the server decided to send this message (see "Timestamps")
  "data": { "...": "message-type-specific payload, see below" }
}
```

### Message types

| Type | Sent | `data` payload |
|---|---|---|
| `PERCEPTION_FRAME` | Once per scan (whenever `publish()` is called with one) | Full perception payload -- see "Perception frame" below. |
| `HEARTBEAT` | Every `streaming_heartbeat_interval_s` (default 2s), regardless of whether a frame was also published in that window | `{uptime_s, frames_sent, clients_connected}` |
| `SYSTEM_STATUS` | Once per client connection | `{status, source_id, scan_rate_hz}` |
| `ERROR` | When the server catches a recoverable internal error (e.g. one scan's pipeline run raised) | `{code, message}` |

**No `RAW_LIDAR` message type** on this channel, unlike some illustrative protocol sketches --
the legacy raw `<START>`/`<END>` line protocol (see "Legacy raw protocol") is a deliberately
separate, unchanged wire format on its own port, not multiplexed into this JSON envelope. A
client that only wants points can already get them from a `PERCEPTION_FRAME`'s optional
`data.points` field (see "Raw point data") without a redundant second points-only message type.

### Perception frame

`data` for a `PERCEPTION_FRAME` message (built by
`perception/src/serialization/unity_protocol.py`; unchanged in shape from Phase 11 except that its
own `protocol_version` field moved to the envelope, and `clearance` now carries real data once the
Phase 10 clearance engine landed -- see docs/collision.md "Directional clearance" for the schema
this field's contents come from):

```jsonc
{
  "timestamp": 1234567890.123,
  "scan_id": "uuid-string",
  "sequence_number": 42,                 // same value as the envelope's frame_id
  "source_id": "simulated:08_approaching_obstacle",

  "objects": [
    {
      "track_id": "track-3", "classification": "vehicle_like", "confidence": 0.86,
      "centroid": {"x": 6.2, "y": 0.1}, "width": 1.8, "depth": 4.2, "distance": 6.2,
      "velocity": {"vx": -1.8, "vy": 0.0},          // null until tracking reports a reliable estimate
      "direction": 182.5,                             // null under the same condition
      "predicted_position": {"x": 6.0, "y": 0.1},      // null under the same condition
      "tracking_state": "confirmed", "movement_state": "moving",
      "track_age": 12, "track_hits": 12, "track_misses": 0,
      "point_count": 18, "aspect_ratio": 2.3           // real geometric features (DetectedObject.point_count /
                                                          // shape_features.aspect_ratio), null if not populated
    }
  ],

  "risk": {                                           // null if the collision stage isn't wired in
    "overall_risk": "warning",
    "most_critical": { "track_id": "track-3", "...": "one results[] entry" },
    "results": [ { "track_id": "...", "distance": 6.2, "ttc": 2.8, "risk_level": "warning", "...": "..." } ]
  },

  "clearance": {                                        // null if the clearance stage isn't wired in (see docs/collision.md "Directional clearance")
    "front": {"direction": "front", "distance_m": 4.08, "nearest_point": {"x": 7.33, "y": 0.26}},
    "rear": {"direction": "rear", "distance_m": 5.73, "nearest_point": null},   // null nearest_point = no return in that quadrant within range
    "left": {"direction": "left", "distance_m": 7.29, "nearest_point": {"x": 8.49, "y": 8.49}},
    "right": {"direction": "right", "distance_m": 7.29, "nearest_point": null},
    "min_clearance_m": 4.08, "min_direction": "front", "corridor_width_m": 16.97,
    "overall_status": "safe", "reason": ["Closest clearance is 4.08m, front."]
  },

  "vehicle": {"x": 0.0, "y": 0.0, "heading": 0.0, "speed_mps": 0.0},

  "config": {                                          // vehicle geometry + risk thresholds -- Unity never hard-codes these
    "vehicle_length_m": 4.5, "vehicle_width_m": 1.8,
    "front_safety_margin_m": 1.0, "rear_safety_margin_m": 0.5,
    "left_safety_margin_m": 0.3, "right_safety_margin_m": 0.3,
    "collision_warning_distance_m": 5.0, "collision_critical_distance_m": 2.0,
    "collision_warning_ttc_s": 4.0, "collision_critical_ttc_s": 2.0,
    "lidar_range_max_m": 12.0
  },

  "map": null,                                         // present only every streaming_map_every_n_scans (default 5)
  "points": null                                        // present only when a point_mode other than "none" is requested
}
```

**The exact schema follows the actual Python models, nothing invented**: every field traces
directly to `models.tracking.TrackedScan`, `models.collision.CollisionAssessment`,
`models.clearance.ClearanceAssessment`, `models.mapping.OccupancyGrid`,
`models.collision.VehicleState`, or `common.config.Settings` -- see
`perception/src/serialization/unity_protocol.py`'s own per-field docstrings for the exact source
of each one.

### Example valid message (full, one PERCEPTION_FRAME)

See "Perception frame" above -- that block, wrapped in the envelope shown in "Message envelope,"
*is* a complete, valid example message. A minimal valid message (no objects/risk/map/points) is
also completely valid:

```json
{"protocol_version": "2.0.0", "message_type": "PERCEPTION_FRAME", "frame_id": 0, "timestamp": 1000.0, "transmission_timestamp": 1000.001, "data": {"timestamp": 1000.0, "scan_id": "s", "sequence_number": 0, "source_id": "simulated:01_empty", "objects": [], "risk": null, "clearance": null, "vehicle": {"x": 0.0, "y": 0.0, "heading": 0.0, "speed_mps": 0.0}, "config": {"vehicle_length_m": 4.5, "vehicle_width_m": 1.8, "front_safety_margin_m": 1.0, "rear_safety_margin_m": 0.5, "left_safety_margin_m": 0.3, "right_safety_margin_m": 0.3, "collision_warning_distance_m": 5.0, "collision_critical_distance_m": 2.0, "collision_warning_ttc_s": 4.0, "collision_critical_ttc_s": 2.0, "lidar_range_max_m": 12.0}, "map": null, "points": null}}
```

### Versioning policy

`protocol_version` follows semver informally: a breaking change (removed/renamed/reshaped field)
bumps the major component; a purely additive change does not require one. **`2.0.0`** (this
phase) is a breaking reorganization from Phase 11's un-enveloped `1.0.0` messages: top-level
`scan_id`/`source_id`/etc. moved under `data`; `message_type`/`frame_id`/`transmission_timestamp`
were added at the envelope level. Deliberately a code constant
(`perception/src/streaming/protocol.py::PROTOCOL_VERSION`), **not** a `Settings`/env-var field --
see `common/config.py`'s own comment: making it independently configurable would let a
misconfigured value lie about the wire format the code actually implements. A client should log
(not crash) on an unexpected major version.

### Raw point data / point modes

`data.points` (only present when requested -- see "Bandwidth" below) supports three modes
(`Settings.streaming_point_mode`, or `scripts/serve_unity_bridge.py --point-mode`):

| Mode | Entry shape | When to use |
|---|---|---|
| `"polar"` (default) | `{angle, distance, valid}` | Matches the sensor's own native representation and the legacy raw protocol. |
| `"cartesian"` | `{x, y, valid}` | Skips a client-side trig step -- most Unity rendering code wants this directly. |
| `"both"` | `{angle, distance, x, y, valid}` | Debugging/cross-checking both representations at once. |
| (omit / `"none"`) | `data.points` is `null` | Default -- points normally travel over the separate legacy raw port instead (see below), not duplicated here. |

## Legacy raw protocol (unchanged)

```
<START>
angle,distance
angle,distance
...
<END>
```

Byte-compatible with the existing, unmodified `LidarTCPClient.cs`/`LidarSerialReader.cs`, served
on its own port (`Settings.streaming_raw_port`, default `5005`, matching the existing hard-coded
Unity value) by `streaming.server.RawLidarStreamServer` -- a completely independent server from
the structured JSON one, so a client can use either, both, or neither. **Unit contract**: the
wire value is meters × 10 (decimeters) -- `LidarCubes.cs` recovers meters via `distance / 10`,
despite that script's own comment saying "cm"; `format_raw_scan` matches the code Unity actually
runs, not the comment (see docs/unity.md "Legacy raw protocol" for the full derivation).

## Message framing

**Newline-delimited JSON** -- one JSON object per line, matching the same
`StreamReader.ReadLine()`-friendly style the legacy raw protocol and every existing Unity script
already use. Chosen over length-prefixing because it needs no new delimiter scheme to design and
keep in sync between Python and C#; a JSON object's own `{...}` structure is unambiguous once a
full line is available, and the newline only marks "a full line has arrived."

TCP is a byte stream, not a message stream: `perception/src/streaming/framing.py::MessageFramer`
explicitly handles a `recv()` returning less than one line, more than one line, or a line split
across two `recv()` calls -- see `perception/tests/test_streaming_framing.py` for all three cases
exercised directly with synthetic byte chunks (13 tests, no real socket needed). Malformed JSON on
an otherwise-complete line is skipped, not raised -- one corrupt line never loses every
well-formed message queued behind it in the same chunk.

## Occupancy map streaming

**Chosen approach: periodic, downsampled full-map resend** (one of this phase's own explicitly
acceptable options) -- not per-cell change tracking or a separate incremental-diff protocol.
`Settings.streaming_map_downsample` (default `4`) strides the 400×400 grid down to 100×100 before
base64-packing it (~13KB per update instead of ~156KB raw at full resolution);
`Settings.streaming_map_every_n_scans` (default `5`) sends it only every Nth scan, not every one
-- Unity keeps showing the last received map in between. **The underlying Python occupancy map
itself is completely unchanged** -- `mapping.OccupancyGridMapper`'s own internal
400×400-at-0.1m-resolution representation is untouched; downsampling happens only in
`serialization.pack_occupancy_grid`, purely as a wire-format concern.

This was chosen over incremental/changed-cell updates because: (a) it requires no new
client-side state-reconstruction logic (each map message is self-contained and independently
valid, so a client that missed several updates or just connected still gets a complete,
correct map on the very next one -- consistent with this phase's own "the latest state matters
more than a complete history" philosophy for frame dropping), and (b) at the chosen downsample/
interval, actual measured size (~13KB, ~0.11ms average serialization time -- see "Performance")
is already comfortably cheap enough that the added complexity of incremental diffing was not
justified by any measured need -- "do not optimize prematurely," per this phase's own
instruction.

## Non-blocking design

**The perception pipeline never waits for Unity.** `PerceptionStreamServer.publish()`/
`RawLidarStreamServer.publish_scan()` only ever enqueue into each connected client's own
`LatestFrameQueue` (a `deque.append`, microseconds) and return -- neither ever calls a blocking
socket operation itself. Each client gets its own dedicated sender thread; that thread is the
*only* code path that ever calls `socket.sendall()`, so one slow/stalled client can never stall
another client, let alone the caller of `publish()`. Measured directly: 200 `publish()` calls
against a client that never reads a single byte completed in under 1ms total (see
`perception/tests/test_streaming_server.py::TestNonBlockingPublish` and "Performance" below for
the actual number).

## Frame dropping

Each client's `LatestFrameQueue` (`Settings.streaming_max_outgoing_queue`, default `2`) is
bounded and **evicts the oldest queued message to make room for the newest** once full -- never
the other way around, and `put()` never blocks or raises. `2` (not `1`) leaves room for a
`HEARTBEAT` to queue up alongside one still-pending `PERCEPTION_FRAME` without evicting either
immediately, while still bounding memory to a handful of messages. **This never drops a message
silently in a way that loses safety information permanently**: `ERROR` messages and every
`PERCEPTION_FRAME`'s *current* risk/TTC state are always about the *latest* system state -- a
dropped older frame's risk assessment is, by construction, already superseded by whatever the
next (still-to-be-delivered) frame says, which is the entire point of "the newest state matters
more than a complete history" for real-time visualization. `LatestFrameQueue.dropped_count`
tracks eviction counts per client for logging/diagnostics.

## Python API

```python
from streaming import PerceptionStreamServer, RawLidarStreamServer
from streaming.protocol import build_perception_frame_message

json_server = PerceptionStreamServer()   # reads defaults from common.config.Settings
raw_server = RawLidarStreamServer()
json_server.start()
raw_server.start()

for tracked_scan in ...:
    message = build_perception_frame_message(tracked_scan, collision_assessment=..., settings=settings)
    json_server.publish(message)          # never blocks
    raw_server.publish_scan(cartesian_scan.points)

json_server.stop()
raw_server.stop()
```

Not tightly coupled to any one perception module: `publish()` takes an already-built message
dict, so a caller assembles whatever it has wired up (see `scripts/serve_unity_bridge.py`) from
whatever pipeline stages it chooses to run.

## Heartbeat / connection status

The server sends a `HEARTBEAT` every `streaming_heartbeat_interval_s` (default 2s) *even when no
new `PERCEPTION_FRAME` was published in that window*, so a client can distinguish "no new scan
yet" from "the server has stopped talking to me." `PerceptionTCPClient.cs` tracks the time of the
last message received (frame *or* heartbeat) and reports:

- `Connected` -- receiving normally.
- `Stale` -- socket still open, but nothing received for `connectionTimeoutSeconds` (default 6s
  -- several heartbeat intervals, not a tight 1× multiple, to absorb ordinary local scheduling
  jitter without flapping). Distinct from a clean disconnect.
- `Reconnecting` / `Disconnected` -- the socket itself closed or errored.

The Unity HUD (`HUDController.cs`) displays all four states with distinct text/color.

## Timestamps

**Two timestamps, deliberately not three** (per this phase's own "avoid unnecessary complexity if
the existing system only supports one [additional] timestamp"): `timestamp` (the scan's own
capture/processing time, `TrackedScan.timestamp`) and `transmission_timestamp` (stamped at
`publish()` call time, before any per-client queueing). A third "Unity receive timestamp" is not
part of the wire protocol -- the *receiver* stamps its own arrival time locally (see "Latency
measurement") rather than Python trying to predict or embed it.

`transmission_timestamp` is set once, at publish time, not re-stamped per client at actual
socket-write time: for a healthy connection (queue never backing up) these are effectively the
same moment; if the queue *is* backed up, that queueing delay is itself latency worth surfacing,
so folding it into `transmission_timestamp` rather than separately tracking "time actually on the
wire" is a deliberate simplification, not an oversight.

## Frame IDs

`frame_id` (envelope-level) is `TrackedScan.sequence_number` itself -- not a second, independent
counter; there is exactly one meaningful per-scan sequence number in this system already
(`tracking.ObjectTracker`'s own monotonic counter, see docs/tracking.md), reused rather than
duplicated.

`streaming.protocol.classify_frame_id(last_frame_id, new_frame_id)` (mirrored exactly in Unity's
`FrameIdValidator.cs`) classifies every incoming frame:

- **Accept**: first frame ever seen, or exactly `last + 1`.
- **Gap**: newer than `last`, but one or more `frame_id`s were skipped -- **still accepted**.
  Real-time visualization needs the current state, not a guarantee every historical frame was
  seen -- the same principle `LatestFrameQueue`'s drop-oldest policy already applies on the
  sending side.
- **Duplicate** (`== last`) / **Out-of-order** (`< last`): **rejected**. A real-time renderer must
  never let an older or repeated frame overwrite a newer one already applied.

`PerceptionTCPClient.cs` performs this check *before* firing `OnPerceptionFrameReceived` --
rejected frames never reach any downstream visualization script at all, so
`TrackedObjectVisualizer`/`OccupancyMapRenderer`/etc. never need their own ordering logic.

## Latency measurement

**Local-clock assumption**: Python and Unity run on the same machine in this project's current
scope, sharing one clock -- so `arrival_time - transmission_timestamp` is a direct, valid latency
measurement with no clock-synchronization concern. This would **not** hold across two different
machines without NTP or an explicit clock-offset calibration; documented, not solved, since
nothing in this project's scope so far spans more than one machine.

`scripts/benchmark_streaming.py` measures this directly with a real local TCP client (mirroring
`PerceptionTCPClient.cs`'s own background-thread-read design) -- see "Performance" for the
measured avg/p95/max numbers. Cannot measure the remaining "Unity receive" / "Unity render" legs
of true end-to-end latency without a live Unity Editor (see "Status").

## Security / validation

This is a local prototype, not a production/networked-untrusted-peer system --
**local TCP on `127.0.0.1` is not production-secure over an untrusted network**, and no
authentication is implemented (none was required by the existing architecture). Still validated:

- `Settings.streaming_max_message_bytes` (default 4MB) -- an outgoing message exceeding this is
  dropped with a logged warning rather than sent (a legitimate frame, even full-resolution
  undownsampled map, never approaches this; exceeding it indicates a bug or malicious input).
- Every incoming line is size- and shape-checked before being trusted (`MessageFramer`,
  `PerceptionTCPClient`'s try/catch around every deserialize call).
- No `Instantiate`/unbounded allocation is ever driven directly by an unvalidated field (e.g.
  `objects` array length isn't trusted to preallocate anything -- Unity iterates and creates
  views one at a time, naturally bounded by how many distinct `track_id`s actually appear).

## Data validation

Handled throughout, not bolted on afterward -- see docs/unity.md "Data validation" for the full
Unity-side list (unchanged from Phase 11, still accurate); this phase adds: unknown
`message_type` is logged and ignored (not an error); a `PERCEPTION_FRAME` missing `frame_id` is
dropped with a warning; malformed JSON on either the framing layer (Python) or `JsonConvert.
DeserializeObject` (Unity) is caught and skipped, never propagated as a crash.

## Reconnection

`PerceptionTCPClient.cs` retries every `reconnectDelaySeconds` (default 2s,
`Settings.streaming_reconnect_interval_s` documents the same value) after any disconnect, and
never opens a second connection while one is already open/connecting (a single `ReadLoop`
background thread per component instance, by construction). The server side requires no special
handling for "Python starts after Unity" vs. "Unity starts after Python": the listener socket
exists and accepts connections independently of when (or whether) a client ever connects, and
`publish()` with zero connected clients is simply a no-op per client (see "Testing").

## Logging

`[STREAM]`-prefixed log lines at `INFO` for connect/disconnect/server-start-stop, `WARNING` for
dropped/oversized/malformed messages -- never per-point or per-full-frame in normal operation (no
raw point or full JSON frame is ever logged by default). `common.logging`'s existing
`DEBUG`-level configuration surfaces more detail when needed (unchanged mechanism from every
earlier phase).

## Configuration

All in `common.config.Settings`, `LIDAR_`-prefixed env-var overridable:

| Setting | Default | Meaning |
|---|---|---|
| `streaming_host` | `127.0.0.1` | Bind address for both servers. |
| `streaming_json_port` | `5006` | Structured JSON protocol port. |
| `streaming_raw_port` | `5005` | Legacy raw protocol port (matches `LidarTCPClient.cs`'s existing default). |
| `streaming_heartbeat_interval_s` | `2.0` | Seconds between `HEARTBEAT` messages. |
| `streaming_connection_timeout_s` | `6.0` | Seconds with no message before a client reports `Stale`. |
| `streaming_reconnect_interval_s` | `2.0` | Unity-side reconnect attempt spacing (documented here; Python's server doesn't read it itself). |
| `streaming_max_outgoing_queue` | `2` | Per-client bounded queue depth. |
| `streaming_point_mode` | `"polar"` | `"none"` \| `"polar"` \| `"cartesian"` \| `"both"` for `data.points`. |
| `streaming_map_every_n_scans` | `5` | How often the occupancy map is included. |
| `streaming_map_downsample` | `4` | Map downsample stride. |
| `streaming_max_message_bytes` | `4_000_000` | Outgoing message size ceiling. |

`protocol_version` is deliberately **not** here -- see "Versioning policy."

## Performance

`scripts/benchmark_streaming.py`, run against this implementation:

### Serialization time and message size

| Scenario | Serialize avg | Size |
|---|---|---|
| Perception frame, no points, no objects | 0.016 ms | 757 B |
| Perception frame, 360 points | 0.38 ms | 18.6 KB |
| Perception frame, 10 objects | 0.12 ms | 7.7 KB |
| Perception frame, 50 objects | 0.65 ms | 34.2 KB |
| Occupancy map (downsample=4, as streamed) | 0.12 ms | 14.2 KB |
| Occupancy map (full resolution, for comparison) | 1.25 ms | 214 KB |
| Maximum realistic frame (50 objects + 360 points + map) | 1.11 ms | 65.6 KB |
| Raw LiDAR protocol, 360 points | 1.06 ms | 4.6 KB |

### Load test: throughput, dropped frames, end-to-end latency

At the realistic operating range this project actually targets (10-30Hz, 1-50 objects -- typical
scenarios have 1-5), **zero dropped frames**, sub-6ms max latency:

| Rate | Objects | Published | Received | Dropped | Avg latency | P95 | Max |
|---|---|---|---|---|---|---|---|
| 10Hz | 1 | 20 | 20 | 0 | 0.20ms | 1.95ms | 2.01ms |
| 10Hz | 50 | 20 | 20 | 0 | 1.10ms | 2.02ms | 2.02ms |
| 20Hz | 50 | 40 | 40 | 0 | 1.28ms | 3.11ms | 5.64ms |
| 30Hz | 50 | 56 | 56 | 0 | 1.36ms | 3.95ms | 4.99ms |

**Degradation only found well beyond any realistic scenario** -- at 200Hz (20× the 10Hz target)
with 200 objects (this project's simulator scenarios use 1-5), throughput itself dropped (362 → 259
published, since serializing 200-object messages started taking longer than the 5ms publish
period) and 75/259 delivered frames were dropped, with p95 latency at 56ms. This is expected,
honest "do not optimize prematurely" behavior: found by measuring, not assumed, and nowhere near
this project's actual real-time requirement.

**Not measurable in this environment**: Unity-side parsing/processing time, render FPS, and
memory -- see docs/unity.md "Performance"/"Status."

## Testing

New: 13 framing + 22 protocol + 14 queue + 20 server (real-socket integration) = **69 new Python
tests**, plus Unity `FrameIdValidatorTests.cs`/`PerceptionProtocolParsingTests.cs` (written,
**not run** -- no Unity Editor available). All previous Phase 1-11 Python tests continue passing
unmodified -- see the completion report for exact totals.

## Serial path

`LidarSerialReader.cs` (unmodified) remains the sole owner of the physical serial COM-port
connection when `LidarInputManager.mode == LegacySerial` -- exactly as in Phase 11; nothing in
this phase introduces a second, competing serial reader. The structured JSON protocol has no
serial transport of its own (it is inherently a Python-perception-pipeline output, not something
raw serial hardware could produce -- see docs/unity.md "Live scan update").

## Do not implement yet

Mobile app, MQTT, camera/radar fusion, ML, SLAM, vehicle control, authentication, production cloud
deployment -- out of scope for this project's current stage.

**Local cloud backend + dashboard have since been added** (`cloud/backend`, `cloud/dashboard`) as
a second, independent TCP client of `PerceptionStreamServer` -- exactly the same relationship
Unity has to it, just a different consumer, not a Python->Unity->dashboard relay. This was always
architecturally possible without a protocol change (`PerceptionStreamServer.publish()`'s plain-dict
input, and the versioned envelope itself, are transport- and consumer-agnostic, and the server
already supported multiple simultaneous clients) -- see docs/cloud.md for that layer's own
documentation.

## Known limitations

- **Unity side not compiled/run in a live Editor** -- see "Status."
- **Local-clock latency assumption** -- see "Latency measurement"; would need NTP/calibration
  across separate machines.
- **No authentication/encryption** -- local-prototype scope, explicitly documented, not a gap to
  silently work around.
- **Degradation exists at extreme, unrealistic load** (200Hz/200 objects) -- documented, not a
  concern at this project's actual real-time target.
