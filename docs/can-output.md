# STM32 → Vehicle-ECU CAN Output (Phase 5)

## Status

**Implemented: a software model of the STM32's CAN-transmit stage** — a configurable message/
signal definition, the `STM32ProcessedFrame → CAN` semantic mapping, per-signal validation,
transmission control (periodic + event, rate-capped, sequenced), error handling + recovery +
health, a standard DBC-style bit-packer (used only once a real layout is supplied), and two
software transports (`MockCANTransport` / `LoggingCANTransport`, both **SIMULATED CAN OUTPUT** —
no bus required).

**Not implemented / PENDING the hardware team:** every real CAN parameter — bitrate, CAN IDs,
standard/extended, DLC, byte layout, endianness, scaling, signed-ness, signal offsets/lengths,
checksum choice, transmission periods. All default to `None` and are reported by
`CANOutputConfig.pending_hardware_parameters()`. **Nothing is invented.**

**No physical CAN validation.** In the real vehicle this logic runs on the STM32, not the Edge.
The Edge model exists so the design, validation, and error handling are exercised now and so the
firmware team can adopt the message catalogue / mapping directly.

## Architecture

```
A3M1 ─UART─►┐
            ├─► STM32: perception + fusion ─► Processed State (STM32ProcessedFrame, Phase-2 contract)
R121 ─CAN──►┘                                        │
                                    ┌───────────────┴────────────────┐
                                    ▼                                ▼
                       can_output (this package)          datasources.esp32 (Phase 3)
                       STM32CANOutput ─► CANTransport      ESP32Source ─► LiveState ─► Dashboard/Unity/PG
                       ─► Vehicle ECU
```

* `can_output` imports **nothing** from `datasources.esp32` (asserted by test). The ESP32 path
  runs identically whether or not CAN output is enabled, and a CAN failure is caught before it
  can touch the ESP32 path (requirement 8).
* Both outputs consume the **same** `STM32ProcessedFrame`.

## Package (`perception/src/can_output/`)

| File | Role |
|---|---|
| `errors.py` | `CANOutputError` / `CANConfigurationError` / `CANValidationError` / `CANBusError` / `CANTransmitError`. |
| `message_spec.py` | `CANSignalSpec` / `CANMessageSpec` / `CANOutputConfig` (every hardware field `None`/PENDING); `build_default_message_catalog()` (the **proposed, not final** message set); the PROPOSED `*_CODES` enum→ordinal maps. |
| `payload.py` | `extract_payloads(STM32ProcessedFrame)` → `PerceptionHeaderPayload` / `ObjectStatePayload×N` / `SafetyStatePayload`; `validate_header` / `validate_object_state` / `validate_safety_state`. |
| `frame.py` | `CANFrame` — encoded (real bytes + CAN ID) or unencoded (semantic signals only, layout PENDING). |
| `encoder.py` | `SignalPacker` (standard start-bit/length/byte-order/scale/offset/signed DBC packing — a *mechanism*); `CANFrameEncoder.encode_frame()` (assembles + validates the message set for one frame). |
| `transport.py` | `CANTransport` ABC; `MockCANTransport` (tests, injectable failures); `LoggingCANTransport` (`[SIM-CAN]` log sink); `build_can_transport(settings)`. |
| `output.py` | `STM32CANOutput` — `start` / `submit(frame)` / `tick(now)` / `stop`; `CANOutputState`, `CANOutputHealth`. |
| `__init__.py` | exports + `build_can_output(settings)`. |

## CAN message abstraction

A `CANMessageSpec` is fully configurable. What is **fixed here (software design)**:

| Field | Meaning |
|---|---|
| `name`, `description` | semantic message identity, e.g. `OBJECT_STATE` |
| `signals[*].name`, `.source` | which processed-perception field feeds each signal |
| `transmission_mode` | `periodic` / `event` / `periodic_and_event` |
| `multiplexed`, `max_multiplex_count` | one instance per tracked object, bus-load cap |

What is **PENDING the DBC** (`None` until provided): `can_id`, `extended_id`, `dlc`,
`cycle_time_ms`, `checksum_algorithm`, and each signal's `start_bit` / `length_bits` /
`byte_order` / `scaling(scale/offset/signed/min/max)`. `CANOutputConfig` additionally needs
`bitrate_bps` and `interface`.

`STM32CANOutput` produces **unencoded** `CANFrame`s (semantic signal values, no bytes) until a
spec is fully specified; then the identical frames come out **bit-packed** with real CAN IDs —
no redesign (requirement 11). `CANOutputConfig.require_fully_specified = True` makes an
incomplete config a hard `CANConfigurationError` at `start()`.

### Proposed (not final) message catalogue

| Message | Mode | Signals (semantic sources) |
|---|---|---|
| `PERCEPTION_HEADER` | periodic | `sequence_number`, `frame_id`, `timestamp_ms`, `active_object_count`, `lidar_ok`, `radar_ok`, `fusion_active`, `stm32_ok`, `rolling_counter`, `checksum` |
| `OBJECT_STATE` (multiplexed by `object_index`, cap 16) | periodic | `object_index`, `track_id_hash` (16-bit crc32 of the string id), `object_type` (code), `distance_m`, `relative_velocity_mps`, `confidence`, `object_ttc_s` (`-1` = not closing), `object_risk` (code), `fusion_source` (0/1/2), `rolling_counter`, `checksum` |
| `SAFETY_STATE` | periodic **and** event (immediate on `overall_risk` change) | `overall_risk` (code), `collision_predicted`, `most_critical_ttc_s`, `min_clearance_m`, `min_clearance_direction` (code), `clearance_state` (code), `front/rear/left/right_clearance_m`, `rolling_counter`, `checksum` |

Enum→code maps (`OBJECT_TYPE_CODES` = unknown 0 / wall 1 / pole_like 2 / vehicle_like 3 /
person_like 4 / large_obstacle 5; `RISK_CODES` = safe 0 / warning 1 / critical 2 ascending;
`CLEARANCE_DIRECTION_CODES`; `CLEARANCE_STATE_CODES`) are **PROPOSED** — one edit in
`message_spec.py` remaps them to the real DBC.

## Fields supported

object/track ID (`track_id_hash` + the full string on the ESP32 path), object type, distance,
relative velocity (component + magnitude), confidence, per-object TTC, per-object risk, fusion
source, overall risk, collision-predicted, most-critical TTC, min clearance + direction +
four-way clearance, clearance state, sequence number (perception frame's + the output's own
rolling tx counter), timestamp, sensor/system health (lidar/radar/fusion/stm32 ok).

## Transmission control

* **periodic** — a message is due every `cycle_time_ms` (or `config.default_cycle_time_ms`);
  `tick()` emits only what is due.
* **event** — `SAFETY_STATE` is flagged for immediate send whenever `overall_risk` changes.
* **rate cap** — `config.max_transmit_rate_hz` (default 100, an Edge guard, **not** a bus
  parameter): `tick()` sends `0` until `1/rate` s has elapsed since the last actual send, then
  one per elapsed interval, hard-capped at ~one second's worth. Never "indefinitely at an
  arbitrary rate".
* **sequence** — a rolling tx counter, wrapping at `config.sequence_modulus` (PENDING; `None` =
  no wrap). Every message of one `submit()` shares that submit's counter value.
* **bounded queue** — `config.queue_max_frames` (default 64); overflow drops the oldest and is
  counted in `CANOutputHealth.frames_dropped`.

## Data validation (requirement 6)

Before any frame is queued, `CANFrameEncoder.encode_frame()` runs `validate_*` on each message:
non-empty track id; finite distance ≥ 0 (≤ 1000 m plausibility); finite velocity (≤ 200 m/s);
TTC `None` or finite ≥ 0 (≤ 3600 s); finite clearance ≥ 0; `min_clearance_m` consistent with the
four directions; risk ∈ `RiskLevel`; confidence ∈ [0, 1]; non-negative integer sequence /
timestamp-ms ≥ ~2001. A failing message is **dropped and counted** (`validation_failures`),
never silently transmitted; the frame's other messages still go.

## CAN error handling (requirement 7)

| Condition | Handling |
|---|---|
| CAN unavailable at `start()` | state `UNAVAILABLE`, recovery scheduled, no frames sent |
| bus-off / link failure during send | state `BUS_OFF`, `bus_off_events++`, transport closed, exponential-backoff recovery (`bus_recovery_initial/max_backoff_s`); the un-sent frame is kept |
| single-frame transmit failure | `tx_errors++`, frame re-queued once then dropped, stays `ACTIVE` |
| queue overflow | oldest dropped, `frames_dropped++` |
| recovery | on backoff expiry, reopen the transport; success → `READY`/`ACTIVE`, backoff reset; failure → reschedule, `reconnect_attempts++` |

`STM32CANOutput.health` → `CANOutputHealth(state, reason, encoded, frames_built, frames_sent,
frames_dropped, validation_failures, tx_errors, bus_off_events, reconnect_attempts, queue_depth,
last_tx_at, last_error, pending_hardware_parameters)`.

## Simulation support (requirement 9)

Simulation mode is completely unaffected — it has no STM32, so no CAN output. The CAN model is
`can_output_enabled = False` by default and needs no physical CAN interface ever. When enabled,
`can_output_backend` picks a **software** sink: `mock` (records frames, `MockCANTransport`) or
`logging` (`[SIM-CAN]` log lines). Both log `*** SIMULATED CAN OUTPUT — NOT a real CAN bus ***`
on open.

Optional runtime demonstration: in hardware/mock ESP32 mode, `LIDAR_CAN_OUTPUT_ENABLED=true`
attaches the CAN model to `run_esp32_edge`, fed the same received `STM32ProcessedFrame`s — a
concrete view of the dual-output split. It is fully isolated (`try/except`), so it can never
affect the ESP32 → LiveState → Dashboard/Unity/PostgreSQL path.

## Configuration (`Settings.can_output_*`, env `LIDAR_CAN_OUTPUT_*`)

| Variable | Default | Notes |
|---|---|---|
| `LIDAR_CAN_OUTPUT_ENABLED` | `false` | the CAN model runs only when explicitly enabled |
| `LIDAR_CAN_OUTPUT_BACKEND` | `mock` | `mock` (SIMULATED) \| `logging` — a real name is refused |
| `LIDAR_CAN_OUTPUT_BITRATE_BPS` | *(unset — PENDING)* | e.g. 250000 / 500000 |
| `LIDAR_CAN_OUTPUT_INTERFACE` | *(unset — PENDING)* | e.g. `can0`, `PCAN_USBBUS1` |
| `LIDAR_CAN_OUTPUT_DEFAULT_CYCLE_TIME_MS` | *(unset — PENDING)* | fallback tx period |
| `LIDAR_CAN_OUTPUT_CHECKSUM_ALGORITHM` | *(unset — PENDING)* | `none`\|`xor8`\|`sum8`\|`crc8`\|`crc16_ccitt` — *which* is unknown; the algorithms are reused from `datasources.stm32.crc`, not invented |
| `LIDAR_CAN_OUTPUT_SEQUENCE_MODULUS` | *(unset — PENDING)* | rolling-counter wrap, e.g. 16 |
| `LIDAR_CAN_OUTPUT_MAX_TRANSMIT_RATE_HZ` | `100.0` | Edge guard, not a bus param |
| `LIDAR_CAN_OUTPUT_QUEUE_MAX_FRAMES` | `64` | bounded queue |
| `LIDAR_CAN_OUTPUT_REQUIRE_FULLY_SPECIFIED` | `false` | `true` → `start()` fails until every PENDING field is filled |
| `LIDAR_CAN_OUTPUT_BUS_RECOVERY_INITIAL/MAX_BACKOFF_S` | `1.0` / `10.0` | recovery timing |

Per-message CAN IDs / DLC / byte layout live on the external `CANOutputConfig` message specs
(`build_default_message_catalog()`) — also all `None`/PENDING.

## Hardware parameters still required (before physical CAN testing)

1. CAN **bitrate** (bps) and **interface** name.
2. Per message: **CAN ID**, standard vs **extended** identifier, **DLC**, **transmission period**.
3. Per signal: **start bit**, **length (bits)**, **byte order** (little/big-endian), **scale**,
   **offset**, **signed-ness**, physical **min/max**.
4. **Checksum** algorithm and its signal position (or "none").
5. Rolling-counter width / **sequence modulus**.
6. The real **enum→code** encodings (or confirmation the proposed ones are acceptable).
7. Which messages actually exist on the ECU (the proposed catalogue may be split/merged/renamed).

## Remaining work before physical CAN testing

1. Fill the values above into a `CANOutputConfig` (via `Settings.can_output_*` + per-message
   spec fields, or a loaded DBC) — **configuration only, no code change**.
2. Add a real `can_output.transport.CANTransport` subclass (e.g. `python-can`-backed) and accept
   its name in `build_can_transport()`. `python-can` is **not** currently a dependency.
3. Set `can_output_require_fully_specified = True` and confirm `start()` succeeds (config
   complete) and `CANOutputHealth.encoded` is `True`.
4. Run the message set against an ECU / CAN analyser on a bench rig — not yet performed.
