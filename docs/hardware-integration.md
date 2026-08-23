# Hardware Integration

## Status

**Phase 8: architecture scaffolding complete. Byte-level protocol: not yet known.**

`datasources.STM32Source` (`perception/src/datasources/stm32_source.py`, composing
`perception/src/datasources/stm32/`) is a real, tested implementation of connection management,
framing, CRC validation, sequence validation, dropped-frame detection, timeout detection,
reconnection, and health tracking -- but it has **not been run against real hardware**, and its
LiDAR/radar payload parsers are the always-raising `UnconfiguredLiDARParser`/
`UnconfiguredRadarParser` until the fields below are supplied. `Settings.data_source == "hardware"`
selects it; it never falls back to simulated data.

## Architecture

```
A3M1 --UART--> STM32F103C8T6 --(STM32-to-Edge link)--> STM32Source --SensorFrame--> pipeline
                    ^
                    | CAN
                   R121
```

`STM32Source.connect()` validates every field below via `_validate_protocol_config` and raises:

    STM32ConfigurationError: STM32 protocol configuration incomplete: <missing fields> required.

before ever opening a serial port -- see `perception/src/datasources/stm32/errors.py`.

## Hardware integration checklist

Every row below is a `Settings` field (env var `LIDAR_<NAME>`, see `.env.example`) or a code seam
that is currently a placeholder. Nothing in this list has been guessed; each defaults to `None`
(or a clearly-marked placeholder) until the hardware team supplies the real value.

| # | Item | Setting / seam | Status |
|---|------|-----------------|--------|
| 1 | STM32-to-Edge transport | `stm32_uart_port`, `stm32_uart_baudrate` | Placeholder (`COM3`, `115200`) -- confirm actual port convention (fixed COM#? VID/PID auto-detect?) and real baud rate |
| 2 | Frame boundaries | `stm32_frame_start_marker` / `stm32_frame_end_marker` (hex string) OR `stm32_frame_length_bytes` | **Unset** -- which framing mode, and the actual marker bytes / fixed length |
| 3 | Byte order | `stm32_byte_order` (`"little"` \| `"big"`) | **Unset** |
| 4 | Message-type field | `stm32_message_type_offset`, `stm32_message_type_width` | **Unset** -- confirms the marker+type+seq+payload+crc convention actually holds; if the real framing differs, `datasources/stm32/parsers.py`'s header-decoding call sites need updating, not just config |
| 5 | Message-type IDs | `stm32_lidar_message_type`, `stm32_radar_message_type` | **Unset** |
| 6 | Sequence-number field | `stm32_sequence_number_offset`, `stm32_sequence_number_width`, `stm32_sequence_modulus` | **Unset** -- modulus only needed if the counter wraps (e.g. `65536` for `uint16`) |
| 7 | CRC/checksum algorithm | `stm32_crc_algorithm` (`"none"` \| `"xor8"` \| `"sum8"` \| `"crc8"` \| `"crc16_ccitt"`) | **Unset** -- if the real algorithm isn't one of these, add it to `datasources/stm32/crc.py` |
| 8 | CRC position | *(assumed: trailing bytes of the frame, before any end marker)* | **Assumption, not configured** -- confirm, or extend `STM32Source._verify_crc` if the checksum sits elsewhere (e.g. a separate footer) |
| 9 | LiDAR payload layout | `datasources/stm32/parsers.LiDARMessageParser` (new subclass) | **Not implemented** -- angle/distance field offsets, widths, and scaling (`stm32_lidar_distance_scale`, `stm32_lidar_angle_scale`) |
| 10 | Radar (R121) payload layout | `datasources/stm32/parsers.RadarMessageParser` (new subclass) | **Not implemented** -- range/velocity field offsets, widths, scaling (`stm32_radar_range_scale`, `stm32_radar_velocity_scale`); also needs the R121's own CAN ID(s)/DLC/byte layout as forwarded by the STM32 |
| 11 | Timestamp | `stm32_timestamp_format` (`"unix_epoch_ms"` \| `"unix_epoch_s"` \| `"device_uptime_ms"` \| `"none"`) | **Unset**; even once set, no field offset/width exists yet to actually decode it -- `STM32Source` currently always uses Edge receive time (`_resolve_timestamp`), see that function's docstring |
| 12 | Connection timing | `stm32_connect_timeout_s`, `stm32_read_timeout_s`, `stm32_reconnect_initial_backoff_s`, `stm32_reconnect_max_backoff_s`, `stm32_max_reconnect_attempts` | Have working defaults; tune once real link behavior is observed |
| 13 | Radar → pipeline fusion decision | *(not implemented anywhere)* | **Explicit open question, not a placeholder to fill in** -- `PROJECT_SPECIFICATION.md` ("Important Sensor Limitation") and `pipeline/live_state.py`'s own `"radar": None` comment both currently treat sensor fusion as out of scope for this prototype. `STM32Source.latest_radar_reading` exposes a parsed `RadarReading` (see `models/radar.py`) but nothing consumes it. Confirm with the project owner before wiring it into detection/clustering/tracking/collision/clearance/risk, `sensor_status`, or the dashboard. |

## What NOT to guess

Per this phase's explicit instructions, none of the following were invented, assumed from a
"typical" example, or hard-coded anywhere in `datasources.stm32`: COM port, baud rate, CAN
bitrate, CAN IDs, CAN packet layout, CRC algorithm/polynomial choice, packet length, byte order,
scaling factors, or timestamp epoch/units. Every one is `None`/unset by default and gated behind
`STM32ConfigurationError` at `connect()` time.

## Once the spec is known

1. Fill in the `Settings` fields in the checklist above (`.env`, never hard-coded).
2. Implement a real subclass of `LiDARMessageParser` (and `RadarMessageParser`, if radar fusion is
   approved -- see item 13) in a new module, e.g. `datasources/stm32/parsers_v1.py`.
3. Pass them to `STM32Source(lidar_parser=..., radar_parser=...)` in `scripts/sensor_source.py`'s
   `"hardware"` branch.
4. No other code changes needed -- `connect()`/`read_scan()`/reconnection/health/CRC/sequence
   validation already work against any correctly-configured layout.
5. Run the real hardware validation checklist (A3M1 → STM32 → Edge → detection → classification →
   tracking → TTC → clearance → risk → Dashboard → Unity → PostgreSQL → disconnect/reconnect) on a
   machine actually wired to the rig -- not yet performed, see the verification report for this
   phase.
