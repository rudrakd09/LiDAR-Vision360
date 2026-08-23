# Sensor Fusion (Phase 9)

## Status

**Architecture implemented and tested against synthetic data. Real R121 fusion NOT validated --
the R121 CAN protocol is still unknown (see docs/hardware-integration.md), so nothing here has
ever processed a real radar reading.**

## Why fusion returns a `TrackedScan`, not a new model

`fusion.FusionEngine.fuse(tracked_scan, radar_reading) -> TrackedScan` -- same type in, same type
out. This is the key architectural decision this phase made: rather than invent a parallel
"FusedScan"/"FusedObject" model that collision/clearance/risk/`LiveState`/the dashboard/Unity/
PostgreSQL would all need to be taught to understand, fusion enriches the *existing*
`DetectedObject`/`TrackedScan` types already flowing through every one of those unchanged. A
fused object's `velocity` may be refined and a handful of new `radar_*` fields populated
(`models/objects.py`), but its `object_id`/`track_id`/`classification`/`centroid`/tracking state
are exactly what the existing (untouched) clustering/classification/tracking pipeline already
produced. Concretely, this means requirements 18-20 ("fused objects exposed through LiveState",
"Unity receives the same fused objects", "PostgreSQL receives the resulting tracking/events") are
satisfied for free -- nothing downstream of `tracking.ObjectTracker.update()` needed to change.

## Architecture

```
                         TrackedScan (LiDAR-only, from the existing,
                          unmodified clustering/classification/tracking
                          pipeline)
                                |
RadarReading (models.radar) ---+---> FusionEngine.fuse() ---> TrackedScan (fused)
                                              |
                                    (unchanged from here on)
                                              v
                          CollisionRiskEngine / ClearanceEngine
                                              |
                                              v
                                     LiveStateBuilder
                                       /            \
                                 Dashboard          Unity
                                              |
                                              v
                                       cloud/backend (PostgreSQL)
```

`FusionEngine.fuse()`:

1. **LiDAR-only fallback** (requirement 5, 7) -- if `radar_reading` is `None`,
   `Settings.fusion_enabled` is `False`, the reading is stale (older/newer than
   `fusion_max_timestamp_diff_s` relative to the LiDAR scan -- requirement 9), or every target in
   it fails plausibility (`fusion.validation.is_target_valid` -- requirement 14, the "noisy
   Radar"/"Radar dropout" cases), `fuse()` returns `tracked_scan` completely untouched (the exact
   same object, not a copy).
2. **Association** (`fusion.association.find_associations`, requirements 9-11) -- spatial gating
   (Cartesian distance between a LiDAR object's centroid and a radar target's derived position,
   `fusion_association_max_distance_m`) AND range gating
   (`fusion_association_max_range_diff_m`), greedy nearest-neighbor, one-to-one.
3. **Merge** (requirement 8, 12, 13) -- a matched pair becomes ONE object: the LiDAR object's
   identity/shape/classification/tracking state are untouched; `sensor_sources` gains `"radar"`;
   `radar_target_id`/`radar_confidence`/`radar_range_m` are populated; if the radar reports a
   velocity, the object's `velocity` has its RADIAL component replaced by the radar's range-rate
   (typically far more accurate) while its TANGENTIAL component (from the existing LiDAR/Kalman
   tracker) is preserved -- a standard fusion technique, not a full overwrite.
4. **Radar-only objects** (the "Radar only"/"LiDAR dropout" cases) -- an unmatched radar target
   with an `angle_deg` becomes its own object (`classification=UNKNOWN`, no shape, since radar
   alone provides neither); its `track_id` is `radar-<target_id>` (stable across frames IF the
   radar's own `target_id` is stable -- an external dependency this project cannot verify without
   real hardware) or dropped entirely if no `target_id` was given (no way to preserve identity
   from range/angle alone). A target with no `angle_deg` at all cannot be placed in the plane and
   is dropped, never fabricated a position.
5. **Unmatched LiDAR objects** pass through completely untouched (the "LiDAR+Radar different
   objects" case).

## `RadarReading` stays protocol-independent

`models/radar.py`'s `RadarTarget`/`RadarReading` (Phase 8) are the only radar-shaped input this
package accepts. No CAN ID, DLC, byte offset, or scaling factor is referenced anywhere in
`fusion/` -- everything here operates purely on the already-decoded `range_m`/`angle_deg`/
`velocity_mps`/`confidence`/`target_id` fields, all individually optional except `range_m`. This
is what requirement 1-3 ("do not invent R121 packet fields/CAN IDs", "keep RadarReading
protocol-independent") mean in practice: this package would not need to change at all once a real
`RadarMessageParser` (see docs/hardware-integration.md) starts producing real `RadarReading`s
instead of the synthetic ones every test in this phase uses.

## Configuration (`Settings`, prefix `LIDAR_`)

| Field | Default | Meaning |
|---|---|---|
| `fusion_enabled` | `True` | Master switch -- `False` forces LiDAR-only even with valid radar data. |
| `fusion_association_max_distance_m` | `1.5` | Spatial gate (Cartesian). |
| `fusion_association_max_range_diff_m` | `2.0` | Range gate (independent of the spatial one). |
| `fusion_max_timestamp_diff_s` | `0.5` | Timestamp-based association tolerance. |
| `fusion_max_valid_range_m` | `200.0` | Plausibility bound, not an R121 spec fact. |
| `fusion_max_valid_velocity_mps` | `60.0` | Plausibility bound, not an R121 spec fact. |

None of these are protocol facts -- they are Edge-side tuning knobs for the fusion algorithm
itself, safe to adjust without any hardware-team input, unlike the `stm32_*` fields in
docs/hardware-integration.md.

## Testing

Every test in `perception/tests/test_fusion_*.py` builds synthetic `RadarReading`/`RadarTarget`
objects directly -- no real R121/STM32 data, no claim about the real protocol. See the Phase 9
verification report for the full list and pass counts.

## What real hardware validation still requires

Everything in docs/hardware-integration.md's checklist (a real `RadarMessageParser`), plus, once
that exists:

- Real R121 range/angle/velocity accuracy and update-rate characteristics, to replace the
  currently-placeholder `fusion_association_max_distance_m`/`fusion_max_timestamp_diff_s`/etc.
  defaults with values actually tuned against the real sensor.
- Confirmation the R121 (or the STM32's forwarding of it) provides a stable per-target `target_id`
  across scans -- `FusionEngine`'s track-ID preservation for radar-only objects depends on this.
- The actual sign convention for radar-reported velocity (this project defines and uses
  "negative = closing" internally -- see `RadarTarget.velocity_mps`'s docstring -- but this has
  not been confirmed against the real R121's own convention).
