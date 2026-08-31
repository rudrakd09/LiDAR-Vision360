# ESP32 Gateway Integration (Phase 3)

## Status

**Implemented:** the software path for `DATA_SOURCE=hardware` — a replaceable transport
abstraction, `ESP32Source` (connection management / timeout / reconnect / heartbeat / stale
detection / sequence-gap tracking / malformed-frame rejection), the
`STM32ProcessedFrame → LiveState` adapter (reusing the existing `pipeline.LiveStateBuilder`), and
the hardware edge loop that publishes over the **same** streaming server the Dashboard, Unity, and
PostgreSQL already consume.

**Not implemented (pending the hardware team):** the real ESP32 Wi-Fi transport (IP/port/protocol/
packet format/framing/CRC/auth/encoding). `ESP32Source.connect()` fails loudly
(`ESP32ConfigurationError`, exit code 2) when no transport is configured — it **never** falls back
to simulation data. A development-only `mock` transport (scripted frames, logged loudly as NOT
real hardware) exists so the whole path is runnable and testable now.

**Not touched:** the perception pipeline, STM32 processing logic, and simulation mode. All 10
scenarios and every existing test still pass.

## Architecture

```
A3M1 LiDAR ──UART──►┐
                    ├─► STM32: full perception + fusion ─► STM32ProcessedFrame (Phase-2 contract)
R121 Radar ──CAN───►┘                                              │
                                                     ┌────────────┴───────────┐
                                                     ▼                        ▼
                                               CAN → Vehicle ECU        ESP32 ──Wi-Fi──► Edge PC
                                                (Phase 8)                                   │
                                                                                           ▼
   datasources.esp32.transport.ESP32Transport   ◄──────────────────────────────  (bytes, framed)
                    │  receive() -> one message's bytes
                    ▼
   datasources.esp32.source.ESP32Source
     • JsonProcessedFrameCodec.decode()  -> validated STM32ProcessedFrame  (Phase-2)
     • SequenceValidator (reused from datasources.stm32.sequence)
     • connection / timeout / reconnect-with-backoff / heartbeat / staleness / drop counters
     • NEVER fabricates a frame — poll() returns None + a concrete `status.reason` when unavailable
                    │  poll() -> STM32ProcessedFrame | None
                    ▼
   datasources.esp32.live_state_adapter.ProcessedFrameToLiveState
     • reshape -> TrackedScan / CollisionAssessment / ClearanceAssessment
     • run the EXISTING pipeline.LiveStateBuilder (session id, TrackHistory, events, measured rate)
     • overlay STM32 per-channel sensor_status
                    │  build() -> LiveState (+ intermediates)
                    ▼
   datasources.esp32.edge_runner.run_esp32_edge
     • streaming.protocol.build_perception_frame_message(...)  (UNCHANGED serializer)
     • streaming.PerceptionStreamServer.publish(...)  on Settings.streaming_json_port (5006)
                    │
        ┌───────────┴───────────┬───────────────────────┐
        ▼                       ▼                       ▼
   cloud/backend           Unity PerceptionTCPClient   (both unchanged — they already
   (/ws/live, PostgreSQL)                               consume this exact wire message)
```

`scripts/serve_unity_bridge.py` dispatches on `Settings.data_source`: `"hardware"` →
`run_esp32_edge` (no local perception); `"simulation"` (default) → the unchanged pipeline `run()`.

## Files

| Path | Role |
|---|---|
| `perception/src/datasources/esp32/errors.py` | `ESP32Error` / `ESP32ConfigurationError` / `ESP32TransportError` / `ESP32ConnectionError` (all extend `RuntimeError`). |
| `perception/src/datasources/esp32/transport.py` | `ESP32Transport` ABC; `MockESP32Transport` (tests); `ScriptedScenarioESP32Transport` (`ESP32_TRANSPORT=mock`); `build_transport()`. |
| `perception/src/datasources/esp32/source.py` | `ESP32Source`, `ESP32SourceState`, `ESP32SourceStatus`, `ProcessedFrameSource` ABC. |
| `perception/src/datasources/esp32/live_state_adapter.py` | `ProcessedFrameToLiveState` + pure `processed_frame_to_*` functions. |
| `perception/src/datasources/esp32/edge_runner.py` | `run_esp32_edge(...)` — the hardware edge loop. |
| `perception/tests/test_esp32_{transport,source,live_state_adapter,edge_runner}.py` | 70 tests (incl. the SIMULATED ESP32 TRANSPORT end-to-end test). |

## Connection state machine (`ESP32SourceState`)

| State | Meaning |
|---|---|
| `UNAVAILABLE` | not configured, or reconnection exhausted (`esp32_max_reconnect_attempts`). |
| `CONNECTING` | transport open, waiting for the first valid frame. |
| `CONNECTED` | fresh valid frames flowing (`poll()` may return one). |
| `STALE` | transport open, but the newest frame is older than `esp32_frame_stale_after_s` — `poll()` returns `None`; the old frame is **not** re-served as current. |
| `RECONNECTING` | link dropped; retrying with exponential backoff. |
| `DISCONNECTED` | link closed/errored (transient, pre-reconnect). |

`ESP32SourceStatus` also carries: `reason` (human-readable: *ESP32 disconnected* / *timeout* /
*stale data* / *invalid frame* / *protocol error*), `session_id`, `last_frame_at`,
`last_frame_timestamp`, `last_frame_id`, `last_sequence_number`, `frame_age_s`,
`frames_received`, `frames_rejected`, `dropped_frames`, `sequence_gaps`, `reconnect_attempts`,
`receive_rate_hz`.

## No synthetic fallback

* Transport not configured → `run_esp32_edge` logs the reason and exits **2**. 0 frames published.
* Link never delivers / times out / goes stale / delivers only invalid frames → `ESP32Source.poll()`
  returns `None`; the edge loop publishes a throttled `SYSTEM_STATUS` (`status="hardware_unavailable"`)
  + `ERROR` (`code="HARDWARE_DATA_UNAVAILABLE"`, `message=<reason>`) and **stops publishing
  perception frames** — nothing downstream keeps showing the last hardware frame as current, and
  no simulation data is ever substituted.
* The scripted `mock` transport is reachable **only** via `LIDAR_ESP32_TRANSPORT=mock` (never a
  default) and logs `*** SIMULATED ESP32 TRANSPORT — NOT REAL HARDWARE ***` on every open.

## Session management

`ESP32Source` mints a fresh `session_id` (uuid4) on `connect()` **and on every successful
reconnect**, and clears its own per-session state (sequence expectations, counters, last frame).
`run_esp32_edge` detects the changed `session_id`, calls `ProcessedFrameToLiveState.reset()`
(new `LiveStateBuilder` → track history / event log / last-risk memory cleared), and
`json_server.set_session(...)`. Downstream, the new `session_id` on the wire triggers the
**already-existing** reset logic: `LatestState.reset_for_new_session` (backend), `sessionOrdering`
(dashboard), `SessionValidator` (Unity) — so old objects / tracks / risk / TTC never linger after
a reconnect.

## Debug endpoint

`GET /debug/stream-status` (extended, not duplicated) now also reports `last_error_code` /
`last_error_message` / `last_error_age_ms` — set from the producer's last `ERROR` message (e.g.
`HARDWARE_DATA_UNAVAILABLE` + reason), cleared automatically on the next real perception frame. It
already reported `source_id`, `last_sequence`, `last_timestamp`, `frame_age_ms`, `objects`,
`tracks`, `risk`, `edge_status`, `session_id`. `GET /debug/live-frame` returns the current
`LiveState` frame (source/timestamp/sequence/objects/tracked_objects/ttc/clearance/risk) for any
source.

The dashboard's System Status panel shows an `ESP32` badge (LIVE / STALE / UNAVAILABLE /
CONNECTING) in hardware mode, and a `HARDWARE DATA UNAVAILABLE — <reason>` line when the backend's
last error is `HARDWARE_DATA_UNAVAILABLE`.

## Configuration (`Settings.esp32_*`, env `LIDAR_ESP32_*`)

| Variable | Default | Notes |
|---|---|---|
| `LIDAR_DATA_SOURCE` | `simulation` | set to `hardware` to select the ESP32 path |
| `LIDAR_ESP32_TRANSPORT` | *(unset)* | unset → `connect()` fails loudly. `mock` → scripted dev transport. `tcp`/`udp`/`mqtt`/`websocket`/… **not implemented** (protocol unspecified) — a real transport is a drop-in `ESP32Transport` subclass. |
| `LIDAR_ESP32_HOST` / `LIDAR_ESP32_PORT` | *(unset)* | no IP/port invented |
| `LIDAR_ESP32_AUTH_TOKEN` / `LIDAR_ESP32_PROTOCOL` | *(unset)* | placeholders for a future transport/codec |
| `LIDAR_ESP32_CONNECT_TIMEOUT_S` | `5.0` | operational, not a protocol fact |
| `LIDAR_ESP32_READ_TIMEOUT_S` | `2.0` | per `receive()` |
| `LIDAR_ESP32_RECONNECT_INITIAL_BACKOFF_S` | `1.0` | exponential |
| `LIDAR_ESP32_RECONNECT_MAX_BACKOFF_S` | `10.0` | |
| `LIDAR_ESP32_MAX_RECONNECT_ATTEMPTS` | *(unset = ∞)* | exhausted → `UNAVAILABLE` |
| `LIDAR_ESP32_FRAME_STALE_AFTER_S` | `1.0` | frame age → `STALE` |
| `LIDAR_ESP32_HEARTBEAT_TIMEOUT_S` | `6.0` | no frame this long → timeout + reconnect |
| `LIDAR_ESP32_SEQUENCE_MODULUS` | *(unset)* | e.g. `65536` for a wrapping uint16 |
| `LIDAR_ESP32_MOCK_FRAME_COUNT` | `200` | only used by `ESP32_TRANSPORT=mock` |

## Commands

**Simulation mode (unchanged):**
```powershell
.\scripts\run_demo.ps1 -Scenario 08_approaching_obstacle -Rate 10
```
or directly:
```powershell
python scripts/serve_unity_bridge.py --scenario 08_approaching_obstacle --rate 10
```

**ESP32 / mock mode (SIMULATED ESP32 TRANSPORT — not hardware):**
```powershell
$env:LIDAR_DATA_SOURCE = "hardware"; $env:LIDAR_ESP32_TRANSPORT = "mock"; python scripts/serve_unity_bridge.py --rate 10
```
then start the backend + dashboard as usual (`uvicorn backend.main:app --app-dir cloud/backend/src ...`, `npm run dev`). The dashboard shows `Mode: HARDWARE`, `Source ID: stm32_hardware`, `ESP32: LIVE`, and a scripted approaching-vehicle scenario.

**Real hardware mode (once the hardware team provides the transport spec):**
```powershell
$env:LIDAR_DATA_SOURCE = "hardware"
$env:LIDAR_ESP32_TRANSPORT = "<the real transport name>"
$env:LIDAR_ESP32_HOST = "<ESP32 host>"
$env:LIDAR_ESP32_PORT = "<ESP32 port>"
python scripts/serve_unity_bridge.py --rate 10
```
This requires a real `datasources.esp32.transport.ESP32Transport` subclass registered in
`build_transport()`, plus (if the wire format is not JSON) a real
`datasources.stm32.processed.ProcessedFrameDeserializer` passed to `ESP32Source`. No other code
changes.

## Hardware specifications still required

1. Transport: TCP / UDP / WebSocket / MQTT / raw framed bytes; ESP32 host/port or discovery.
2. Message framing (length prefix / delimiter / one datagram per frame / WebSocket message).
3. Payload encoding: JSON / CBOR / protobuf / fixed binary struct.
4. Per-message CRC/checksum algorithm and position, if any.
5. Auth/encryption, if any (token / PSK / TLS).
6. `metadata.timestamp` epoch + units; `metadata.sequence_number` semantics + wraparound modulus.
7. Heartbeat/keepalive rate; whether the ESP32 emits explicit "no detections" frames or goes silent.
8. Which processed fields the STM32 actually emits (see `docs/stm32-processed-contract.md`).

## Assumptions

1. `DATA_SOURCE=hardware` now selects the **ESP32 processed-frame** path. The legacy raw-serial
   `STM32Source` (Edge runs the full pipeline on raw bytes) remains in the codebase and fully
   tested, but is no longer the hardware path — the target architecture says the Edge must not
   re-interpret raw sensor data.
2. `ESP32Transport.receive()` returns one whole processed-frame message's bytes (the transport
   owns framing, since framing is protocol-specific and unspecified).
3. `poll()` returning a non-`None` frame is itself the "publish this" signal — it only ever
   returns a frame that was fresh, in-order, and valid when read.
4. A transport failure mid-drain does not discard frames already read in that poll (they are
   genuine); the disconnect is surfaced on the next poll.
5. `JsonProcessedFrameCodec` (Phase 2 reference codec) is the default deserializer — replaceable.
6. `LiveState.session_id` is Edge-owned (uuid4 per hardware session), exactly as in simulation.
7. `sequence_number` continuity is checked statelessly per-session via the reused
   `datasources.stm32.sequence.SequenceValidator`; a gap is counted as dropped frames but the
   newest frame is still delivered (real-time visualisation needs current state, not every frame).
8. `LiveState.tracked_objects[].sensor_source` now reflects real per-object attribution
   (`"lidar"` / `"radar"` / `"lidar+radar"`) instead of a hard-coded `"lidar"` — a one-line
   `LiveStateBuilder` change that also fixes the existing fusion path's dashboard label.
```
