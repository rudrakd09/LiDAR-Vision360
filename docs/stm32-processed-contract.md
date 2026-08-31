# STM32 Processed-Perception Contract (Phase 2)

## Status

**Implemented: the protocol-independent internal data model, its schema/semantic validation, its
version gate, and a replaceable serializer/deserializer seam with a reference JSON codec.**

**Not implemented (later phases):** `ESP32Source` (Phase 3), the `STM32ProcessedFrame -> LiveState`
adapter (Phase 3/4), the real STM32↔ESP32 wire codec (pending hardware spec), the STM32→ECU CAN
output interface (Phase 8). No physical hardware has been connected or tested.

## Why this exists

In the target hardware architecture the **STM32F103C8T6 is the primary perception-processing
node**:

```
A3M1 LiDAR ──UART──►┐
                    ├─► STM32: acquisition ─► preprocessing ─► LiDAR/radar processing ─►
R121 Radar ──CAN───►┘        fusion ─► detection ─► classification ─► tracking ─► TTC ─►
                            clearance ─► collision-risk
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                               ▼
              CAN → Vehicle ECU            ESP32 → Wi-Fi → Edge PC
```

The Edge PC receives an **already-processed result**. It must **not** re-run preprocessing,
fusion, detection, tracking, TTC, clearance, or risk. `STM32ProcessedFrame` is the Edge's
internal representation of one such result, deliberately independent of byte layout, CAN IDs, CAN
bitrate, UART baud rate, CRC parameters, and the ESP32 Wi-Fi protocol — all of which remain the
hardware team's to specify.

### Two modes, one `LiveState`

| Mode | Path |
|---|---|
| **Simulation** (unchanged) | `Simulator → existing perception pipeline → LiveState` |
| **Hardware** (new) | `STM32ProcessedFrame → ESP32Source (Phase 3) → LiveState` |

Both converge on the same `models.live_state.LiveState`, which alone feeds **Dashboard, Unity,
and PostgreSQL**. Nothing downstream of `LiveState` changes.

## Where it lives

| Path | Contents |
|---|---|
| `perception/src/models/stm32_processed.py` | The contract data model (`STM32ProcessedFrame` + sub-models). Pure pydantic, no I/O. |
| `perception/src/datasources/stm32/processed/version.py` | Semantic-version parsing + `SUPPORTED_MAJOR_VERSIONS` gate. |
| `perception/src/datasources/stm32/processed/validation.py` | `validate_processed_frame()` — semantic / cross-field / plausibility checks. |
| `perception/src/datasources/stm32/processed/serialization.py` | `ProcessedFrameSerializer` / `ProcessedFrameDeserializer` interfaces + `JsonProcessedFrameCodec` (reference) + `parse_processed_frame()`. |
| `perception/src/datasources/stm32/processed/errors.py` | `STM32ProcessedFrameError`, `STM32ContractVersionError` (both extend `STM32Error`). |
| `perception/tests/test_stm32_processed_contract.py` | 61 tests covering the 13 required cases. |

Import surface: `from datasources.stm32.processed import STM32ProcessedFrame, JsonProcessedFrameCodec, parse_processed_frame, validate_processed_frame, ...`

## Schema

```
STM32ProcessedFrame
├── metadata : STM32FrameMetadata
│   ├── protocol_version : str        semver "MAJOR.MINOR.PATCH"; MAJOR gates compatibility. Default "1.0.0".
│   ├── frame_id : int ≥ 0            STM32's monotonic per-frame id → LiveState.sequence_number
│   ├── sequence_number : int ≥ 0     [HW-SPEC] transport counter; may == frame_id or be independent / wrapping
│   ├── timestamp : float             [HW-SPEC] Unix-epoch SECONDS (deserializer normalises ms/uptime/other epoch)
│   ├── source_id : str               rig identity; default "stm32_hardware" (Settings.hardware_source_id)
│   ├── active_object_count : int ≥ 0 MUST equal len(objects)
│   └── emitter : str                 informational; "stm32"
├── sensor_status : STM32SensorStatus
│   ├── lidar : STM32ChannelHealth               required (primary sensor)
│   │   ├── connected : bool
│   │   ├── ok : bool | None                     channel self-report; None = not reported
│   │   ├── frames_received / frames_dropped : int ≥ 0 | None
│   │   ├── last_update_timestamp : float | None [HW-SPEC] Unix-epoch seconds
│   │   └── detail : str | None                  [HW-SPEC] opaque fault text
│   ├── radar : STM32ChannelHealth | None        None = no R121 wired / not reported (not fabricated)
│   └── fusion_active : bool | None              did fusion combine radar into THIS frame?
├── vehicle_state : VehicleState | None          reused from models.collision (pose + speed_mps); None = no ego-motion source
├── objects : list[STM32TrackedObject]           may be empty; length == metadata.active_object_count
│   └── STM32TrackedObject
│       ├── track_id : str (non-empty, unique in frame)
│       ├── object_type : ObjectClassification   reused enum (wall/vehicle_like/pole_like/person_like/large_obstacle/unknown)
│       ├── position : Point2D                   [HW-SPEC] vehicle-relative metres, forward=+x left=+y (polar→cartesian in deserializer)
│       ├── distance_m : float ≥ 0
│       ├── relative_velocity : Velocity2D | None [HW-SPEC] object−ego m/s; None = not estimated
│       ├── confidence : float [0,1]
│       ├── ttc_s : float ≥ 0 | None             None = not approaching/undefined; 0.0 = overlapping
│       ├── risk : RiskLevel                     reused enum (safe/warning/critical)
│       ├── source : STM32SourceChannel          lidar | radar | fused
│       ├── in_projected_path : bool | None      [HW-SPEC] None = not reported
│       ├── radar_range_m : float ≥ 0 | None     kept alongside distance_m
│       └── radar_target_id : str | None
├── clearance : STM32Clearance | None            None = STM32 does not run/transmit clearance
│   ├── front_m / rear_m / left_m / right_m : float ≥ 0
│   ├── min_clearance_m : float ≥ 0              checked ≈ min(4 directions) within 0.05 m
│   ├── min_direction : ClearanceDirection       reused enum; checked consistent with the minimum
│   ├── status : ClearanceState                  reused enum (safe/caution/low_clearance/critical)
│   └── corridor_width_m : float ≥ 0 | None
├── risk : STM32Risk
│   ├── overall_risk : RiskLevel
│   ├── most_critical_track_id : str | None      if set, MUST name an object in objects[]
│   ├── collision_predicted : bool               if true, predicted_collision_time_s must be set
│   ├── predicted_collision_time_s : float ≥ 0 | None
│   └── reason : list[str]
└── system_status : STM32SystemStatus
    ├── stm32_ok : bool
    ├── processing_load : float [0,1] | None
    ├── processing_time_ms : float ≥ 0 | None    → PerformanceMetrics.pipeline_processing_ms
    ├── can_tx_ok : bool | None                  STM32→ECU CAN path health (informational)
    ├── fault_codes : list[str]                  [HW-SPEC] opaque code vocabulary
    └── notes : str | None
```

`[HW-SPEC]` = the exact representation depends on the hardware team's specification; the field is
normalised by the (replaceable) deserializer to the form documented above. The full list is
`models.stm32_processed.HARDWARE_SPEC_DEPENDENT_FIELDS`.

All models use `extra="ignore"` so a newer-MINOR firmware frame carrying extra fields is accepted
(and its extra fields dropped) by an older Edge build — the semver-minor backward-compatibility
promise.

## Validation

`parse_processed_frame()` / `ProcessedFrameDeserializer.decode()` run three stages and raise a
single `STM32ProcessedFrameError` (or `STM32ContractVersionError`) with a field-naming message —
never a partial/fabricated frame:

1. **Version gate** (`require_supported_version`) — first, before structural parsing, so an
   unknown-MAJOR frame (whose shape may differ) yields a crisp version error.
2. **Structural parse** — pydantic (required fields, types, enum membership, `ge`/`le` bounds).
   `pydantic.ValidationError` is wrapped as `STM32ProcessedFrameError`.
3. **Semantic validation** (`validate_processed_frame`) —
   - finiteness: reject NaN/±inf on any numeric field;
   - timestamp: finite, ≥ `MIN_PLAUSIBLE_EPOCH_S` (2001-09), ≤ now + 60 s;
   - `frame_id` / `sequence_number` / `active_object_count`: non-negative ints; `active_object_count == len(objects)`;
   - object IDs: non-empty and unique within the frame;
   - numeric plausibility: distances/ranges in `[0, 1000] m`, TTC in `[0, 3600] s` (Edge-side sanity bounds, **not** hardware specs — overridable per call);
   - clearance: four distances finite ≥ 0; `min_clearance_m` and `min_direction` consistent with the four values (±0.05 m);
   - risk: valid `RiskLevel`; `most_critical_track_id` (if set) names a present object; `collision_predicted ⇒ predicted_collision_time_s` set.

`decode(..., validate=False)` skips stage 3 only (for already-trusted Edge-internal re-hydration);
stages 1–2 always run.

## Versioning

`metadata.protocol_version` is `MAJOR.MINOR.PATCH`. `SUPPORTED_MAJOR_VERSIONS = {1}`, current
version `1.0.0`.

- Same MAJOR (any MINOR/PATCH) → accepted.
- Unknown/unparseable MAJOR → `STM32ContractVersionError` ("upgrade the Edge software").

A breaking firmware change bumps MAJOR; adopting it means extending `SUPPORTED_MAJOR_VERSIONS`
(and adding a deserializer branch if the payload *shape* changed, not just its field set).

## Serialization seam

```
STM32ProcessedFrame ──Serializer.encode()──► bytes
                    … ESP32 / Wi-Fi transport (NOT defined here) …
bytes ──Deserializer.decode()──► STM32ProcessedFrame   (validated)
```

`ProcessedFrameSerializer` / `ProcessedFrameDeserializer` / `ProcessedFrameCodec` are the abstract
contract. `JsonProcessedFrameCodec` is a **reference** implementation (UTF-8 JSON via pydantic) —
explicitly *not* the hardware wire format; it exists for development, tests, fixtures, logging,
and the Phase 11 simulation-driven bring-up. When the hardware team provides the real framing /
field layout / endianness / CRC, add a new codec class implementing the same two interfaces;
`STM32ProcessedFrame`, the validator, and every consumer stay unchanged.

## Mapping to `LiveState`

Implemented by the Phase 3/4 adapter (not in this phase). Fixed now:

| `LiveState` field | Source | Notes |
|---|---|---|
| `session_id` | ESP32Source per-session `uuid4` | Edge owns session identity (same as simulation) |
| `source_id` | `metadata.source_id` | |
| `timestamp` | `metadata.timestamp` | |
| `sequence_number` | `metadata.frame_id` | |
| `sensor_status["lidar"]` | `sensor_status.lidar` → `SensorChannelStatus(connected=…, point_count=None, valid_percentage=None, mean_distance_m=None)` | STM32 sends no raw point-quality stats → `None`, not fabricated |
| `sensor_status["radar"]` | `sensor_status.radar` → `SensorChannelStatus` or `None` | can be non-`None` in hardware mode (unlike simulation) |
| `objects[]` (`DetectedObject`) | `objects[]` → `object_id`=`track_id`, `centroid`=`position`, `distance`=`distance_m`, `classification`=`object_type`, `velocity`=`relative_velocity`, `sensor_sources` from `source`, `radar_range_m`, `radar_target_id`, `timestamp`=`metadata.timestamp` | `width`/`depth`/`shape_features`/`bounding_box` left `None` |
| `tracked_objects[]` (`TrackedObjectState`) | `objects[]` joined with `risk` → `track_id`, `classification`, `confidence`, `x`/`y`=`position`, `distance`, `velocity`, `sensor_source`=`source.value`, `ttc`=`ttc_s`, `risk`=object `risk`; `first_seen`/`last_seen`/`frames_tracked`/`trajectory` from an Edge-side `TrackHistory` keyed on `track_id` | `tracking_state`/`movement_state` = `None` unless STM32 later sends them |
| `clearance` (`ClearanceAssessment`) | `clearance` → `front/rear/left/right`=`DirectionalClearance(distance_m=…, nearest_point=None)`, `min_clearance_m`, `min_direction`, `corridor_width_m`, `overall_status`=`status`, `reason`, `scan_id/sequence_number/source_id/timestamp` from `metadata` | `None` when `frame.clearance is None` |
| `risk` (`CollisionAssessment`) | `risk` + `objects` → one `CollisionRiskResult` per object (`track_id`, `classification`, `distance`, `ttc`=`ttc_s`, `risk_level`=object `risk`, `in_projected_path`=`in_projected_path or False`, `relative_position`=`position`, `relative_velocity`=…), `overall_risk`, `most_critical_object` by `most_critical_track_id`, `object_count`, `collision_predicted` | fields with no STM32 source use conservative model defaults |
| `events` | detected at the Edge from transitions in the received stream (risk / clearance / track created / lost) — the same "only on actual change" logic `pipeline.LiveStateBuilder` already runs | never a re-computation of perception |
| `performance_metrics` | ESP32Source-measured inter-frame interval + `system_status.processing_time_ms` → `pipeline_processing_ms` | |

Fields with no STM32 source become `None` / model defaults — **never fabricated**. If the
hardware team later provides them (e.g. per-object `tracking_state`, object geometry), extend the
contract with a MINOR version bump and widen the adapter.

## Hardware specifications still required

Nothing below blocks Phases 3–4 (the abstraction and adapter); all are needed only to fill in
real values and run against physical hardware (Phase 12).

1. STM32→ESP32→Edge transport: TCP / UDP / WebSocket / MQTT / raw framed bytes; ESP32 host/port or discovery.
2. Payload encoding: JSON / CBOR / protobuf / fixed binary struct; message boundary convention.
3. `metadata.timestamp` epoch + units (s / ms / device-uptime), and how to align device-uptime to Unix epoch.
4. `metadata.sequence_number` semantics: identical to `frame_id`? independent? wrapping width/modulus?
5. Which per-object fields the firmware actually emits (`relative_velocity` vector vs. scalar closing speed; `in_projected_path`; per-object `tracking_state`/geometry).
6. Coordinate form on the wire: cartesian `(x, y)` vs. polar `(range, bearing)`; angle/heading sign convention.
7. `STM32ChannelHealth.detail` / `STM32SystemStatus.fault_codes` code vocabulary.
8. Per-message CRC/checksum algorithm and position (if any); frame heartbeat/keepalive rate.
9. Whether the STM32 emits explicit "no detections" frames or goes silent.
10. STM32→Vehicle-ECU CAN: IDs, DLC, byte offsets, scaling, endianness, bitrate, message set (Phase 8).

## Assumptions made

1. `protocol_version` is semantic-version; MAJOR is the sole compatibility axis; MINOR/PATCH are additive-only.
2. The STM32 emits fully-processed results; the Edge never recomputes perception in hardware mode (from the Phase 2 brief).
3. One `STM32ProcessedFrame` = one STM32 perception cycle (one fused scan).
4. `metadata.active_object_count` must exactly equal `len(objects)` — the STM32 is authoritative, and a mismatch signals corruption (rejected, not reconciled).
5. `track_id` is a non-empty string, unique within a frame. Cross-frame stability is *expected* but not checked here — that is ESP32Source's stateful concern (Phase 3).
6. Positions are vehicle-relative Cartesian metres, forward=+x / left=+y (project convention, `docs/coordinates.md`); polar input is converted in the deserializer.
7. Timestamps reaching a consumer are Unix-epoch **seconds** (float); the deserializer normalises any other firmware representation first.
8. Plausibility bounds (`MIN_PLAUSIBLE_EPOCH_S`, `MAX_DISTANCE_M = 1000`, `MAX_TTC_S = 3600`, `MAX_CLOCK_SKEW_S = 60`, `CLEARANCE_MIN_TOLERANCE_M = 0.05`) are Edge-side sanity limits, **not** hardware specs, and are overridable per `validate_processed_frame` call (a future caller can wire them to `Settings`).
9. The contract reuses `ObjectClassification`, `Point2D`, `Velocity2D`, `RiskLevel`, `VehicleState`, `ClearanceState`, `ClearanceDirection` verbatim — no parallel enums/types.
10. `JsonProcessedFrameCodec` is a placeholder wire format, explicitly replaceable without touching the model, validator, or consumers.
11. `vehicle_state` absent ⇒ the Edge treats the vehicle as stationary at the origin (the only assumption that never overstates risk), matching `collision_default_vehicle_speed_mps`.
```
