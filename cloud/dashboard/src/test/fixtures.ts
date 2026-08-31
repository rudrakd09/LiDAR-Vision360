/**
 * Real, captured Scenario-08 (`08_approaching_obstacle`) WebSocket payloads -- copied verbatim
 * (minus the 360-entry `points` array, trimmed to `null` for brevity; nothing else changed) from
 * an actual `serve_unity_bridge.py` / backend `/ws/live` run, not hand-typed or invented. See the
 * PR/commit history for the raw capture this was taken from. Using the real wire shape here,
 * rather than a hand-rolled "looks about right" fixture, is the whole point -- a schema-name
 * mismatch between a hand-written fixture and the real payload is exactly the kind of bug these
 * tests exist to catch, and a hand-written fixture could accidentally "fix" that mismatch by
 * construction.
 */
import type { LiveMessage, PerceptionObject, TrackedObjectData, CollisionResult, StreamStatus } from "../types";

/** These fixtures predate `tracked_objects`/`session_id`/`events`/`sensor_status`/
 * `performance_metrics` being added to the wire (see docs/architecture.md "Dashboard and Unity as
 * pure LiveState consumers") -- they cannot be re-captured (the original live run is long gone),
 * so `tracked_objects` below is *derived* mechanically from each fixture's own real `objects`/
 * `risk.results` using the exact same join `pipeline.LiveStateBuilder._tracked_object_states`
 * performs (by `track_id`), not invented. `first_seen`/`last_seen`/`frames_tracked`/`trajectory`
 * are filled from each object's own real `track_age`/`track_hits`/`centroid`/`velocity` fields
 * (already present, captured for real) rather than a plausible-looking guess. */
function deriveTrackedObjects(objects: PerceptionObject[], results: CollisionResult[], timestamp: number, sequenceNumber: number): TrackedObjectData[] {
  const resultByTrack = new Map(results.map((r) => [r.track_id, r]));
  return objects
    .filter((o) => o.track_id)
    .map((o) => {
      const result = resultByTrack.get(o.track_id);
      return {
        track_id: o.track_id,
        classification: o.classification,
        confidence: o.confidence,
        x: o.centroid.x,
        y: o.centroid.y,
        distance: o.distance,
        velocity: o.velocity,
        sensor_source: "lidar",
        first_seen: timestamp,
        last_seen: timestamp,
        frames_tracked: o.track_hits,
        trajectory: [
          {
            frame_id: sequenceNumber, timestamp, x: o.centroid.x, y: o.centroid.y,
            vx: o.velocity?.vx ?? null, vy: o.velocity?.vy ?? null, distance: o.distance,
            classification: o.classification, tracking_state: o.tracking_state,
          },
        ],
        ttc: result?.ttc ?? null,
        risk: result?.risk_level ?? null,
        tracking_state: o.tracking_state,
        movement_state: o.movement_state,
      } satisfies TrackedObjectData;
    });
}

export const REAL_SNAPSHOT_MESSAGE: LiveMessage = {
  type: "snapshot",
  data: {
    connection: {
      state: "connected",
      host: "127.0.0.1",
      port: 5006,
      connected_at: 1786602526.756024,
      last_message_at: 1786602526.9987218,
      frames_received: 3,
      duplicate_or_out_of_order_dropped: 0,
      last_frame_id: 12,
      source_id: "simulated:08_approaching_obstacle",
      scan_rate_hz: null,
      dashboard_clients_connected: 1,
      session_status: "active",
      session_id: "test-session-08",
      session_frames_received: 3,
      measured_scan_rate_hz: null,
    },
    latest_frame: null,
  },
};

const CRITICAL_FRAME_OBJECTS: PerceptionObject[] = [
  {
    track_id: "track-1",
    classification: "vehicle_like",
    confidence: 1.0,
    centroid: { x: 5.1547, y: 0.0002 },
    width: 1.632,
    depth: 0.0324,
    distance: 5.1547,
    velocity: { vx: -2.0035, vy: 0.0005 },
    direction: 179.9857,
    predicted_position: { x: 4.9543, y: 0.0002 },
    tracking_state: "confirmed",
    movement_state: "moving",
    track_age: 14,
    track_hits: 14,
    track_misses: 0,
  },
];

const CRITICAL_FRAME_RESULTS: CollisionResult[] = [
  {
    track_id: "track-1",
    classification: "vehicle_like",
    distance: 5.1547,
    relative_speed: 2.0035,
    in_projected_path: true,
    ttc: 0.8758,
    collision_predicted: true,
    predicted_collision_time: 0.9,
    predicted_collision_position: { x: 3.3516, y: 0.0007 },
    risk_level: "critical",
    risk_score: 1.0,
    reason: ["Predicted footprints intersect in 0.90s (<= critical threshold 2.00s)."],
  },
];

/** frame_id=13, real captured payload -- `risk.overall_risk` is already "critical" with a finite
 * `ttc`, one tracked object, and real (non-round) clearance numbers -- exactly the state the
 * dashboard was reported stuck at "safe/N-A/0" instead of showing. */
export const REAL_CRITICAL_FRAME_MESSAGE: LiveMessage = {
  type: "frame",
  frame_id: 13,
  data: {
    session_id: "test-session-08",
    timestamp: 1786602527.0760803,
    scan_id: "17a71206-da65-413f-8fb4-8da8ec3e30e6",
    sequence_number: 13,
    source_id: "simulated:08_approaching_obstacle",
    objects: CRITICAL_FRAME_OBJECTS,
    tracked_objects: deriveTrackedObjects(CRITICAL_FRAME_OBJECTS, CRITICAL_FRAME_RESULTS, 1786602527.0760803, 13),
    events: [],
    sensor_status: { lidar: { connected: true, point_count: 360, valid_percentage: 100.0, mean_distance_m: 6.5 }, radar: null },
    performance_metrics: { pipeline_processing_ms: 1.2, measured_scan_interval_s: 0.1, measured_scan_rate_hz: 10.0, scans_processed: 13 },
    risk: {
      overall_risk: "critical",
      most_critical: {
        track_id: "track-1",
        classification: "vehicle_like",
        distance: 5.1547,
        relative_speed: 2.0035,
        in_projected_path: true,
        ttc: 0.8758,
        collision_predicted: true,
        predicted_collision_time: 0.9,
        predicted_collision_position: { x: 3.3516, y: 0.0007 },
        risk_level: "critical",
        risk_score: 1.0,
        reason: ["Predicted footprints intersect in 0.90s (<= critical threshold 2.00s)."],
      },
      results: CRITICAL_FRAME_RESULTS,
    },
    clearance: {
      front: { direction: "front", distance_m: 1.894, nearest_point: { x: 5.144, y: -0.8147 } },
      rear: { direction: "rear", distance_m: 5.7313, nearest_point: { x: -8.4813, y: 8.4813 } },
      left: { direction: "left", distance_m: 7.2853, nearest_point: { x: 8.4853, y: 8.4853 } },
      right: { direction: "right", distance_m: 7.2853, nearest_point: { x: -8.4853, y: -8.4853 } },
      min_clearance_m: 1.894,
      min_direction: "front",
      corridor_width_m: 16.9706,
      overall_status: "low_clearance",
      reason: ["Closest clearance is 1.89m, front.", "<= low-clearance threshold 0.60m."],
    },
    vehicle: { x: 0.0, y: 0.0, heading: 0.0, speed_mps: 0.0 },
    config: {
      vehicle_length_m: 4.5,
      vehicle_width_m: 1.8,
      front_safety_margin_m: 1.0,
      rear_safety_margin_m: 0.5,
      left_safety_margin_m: 0.3,
      right_safety_margin_m: 0.3,
      collision_warning_distance_m: 5.0,
      collision_critical_distance_m: 2.0,
      collision_warning_ttc_s: 4.0,
      collision_critical_ttc_s: 2.0,
      lidar_range_max_m: 12.0,
      data_source: "simulation",
    },
    map: null,
    points: null,
  },
};

const NEXT_CRITICAL_FRAME_OBJECTS: PerceptionObject[] = [
  { ...CRITICAL_FRAME_OBJECTS[0], centroid: { x: 4.9548, y: 0.0002 }, distance: 4.9548, track_hits: 15, track_age: 15 },
];
const NEXT_CRITICAL_FRAME_RESULTS: CollisionResult[] = [
  { ...CRITICAL_FRAME_RESULTS[0], distance: 4.9548, ttc: 0.7763, predicted_collision_time: 0.8 },
];

/** frame_id=14, one tick later -- same shape, values moved (distance/ttc/clearance all decreased
 * further), used to prove a newer frame actually replaces the previous one rather than the UI
 * getting stuck on the first frame it ever saw. */
export const REAL_NEXT_CRITICAL_FRAME_MESSAGE: LiveMessage = {
  type: "frame",
  frame_id: 14,
  data: {
    ...REAL_CRITICAL_FRAME_MESSAGE.data,
    sequence_number: 14,
    timestamp: 1786602527.1765504,
    objects: NEXT_CRITICAL_FRAME_OBJECTS,
    // Trajectory carries BOTH points -- this is what the real Edge's `tracking.TrackHistory`
    // would have accumulated by now (frame 13's position, then frame 14's), not just this one
    // frame's -- see deriveTrackedObjects' own docstring for why these fixtures otherwise derive
    // only a single point per frame (they're independent frozen captures, not a live session).
    tracked_objects: deriveTrackedObjects(NEXT_CRITICAL_FRAME_OBJECTS, NEXT_CRITICAL_FRAME_RESULTS, 1786602527.1765504, 14).map((t) => ({
      ...t,
      first_seen: REAL_CRITICAL_FRAME_MESSAGE.data.tracked_objects![0].first_seen,
      trajectory: [...REAL_CRITICAL_FRAME_MESSAGE.data.tracked_objects![0].trajectory, ...t.trajectory],
    })),
    events: [],
    risk: {
      ...REAL_CRITICAL_FRAME_MESSAGE.data.risk!,
      most_critical: {
        ...REAL_CRITICAL_FRAME_MESSAGE.data.risk!.most_critical!,
        distance: 4.9548,
        ttc: 0.7763,
        predicted_collision_time: 0.8,
      },
      results: NEXT_CRITICAL_FRAME_RESULTS,
    },
    clearance: {
      ...REAL_CRITICAL_FRAME_MESSAGE.data.clearance!,
      front: { direction: "front", distance_m: 1.6942, nearest_point: { x: 4.9442, y: -0.8718 } },
      min_clearance_m: 1.6942,
    },
  },
};

const SCENARIO_SWITCH_OBJECTS: PerceptionObject[] = [
  {
    track_id: "track-1",
    classification: "wall",
    confidence: 0.91,
    centroid: { x: 9.0, y: 4.98 },
    width: 4.8, depth: 0.05,
    distance: 10.28,
    velocity: { vx: 0.0, vy: 0.0 },
    direction: 208.0,
    predicted_position: { x: 9.0, y: 4.98 },
    tracking_state: "confirmed", movement_state: "stationary",
    track_age: 1, track_hits: 1, track_misses: 0,
  },
  {
    track_id: "track-9",
    classification: "pole_like",
    confidence: 0.72,
    centroid: { x: 0.0, y: 2.87 },
    width: 0.05, depth: 0.2,
    distance: 2.87,
    velocity: { vx: 0.0, vy: 0.0 },
    direction: 268.0,
    predicted_position: { x: 0.0, y: 2.87 },
    tracking_state: "confirmed", movement_state: "stationary",
    track_age: 1, track_hits: 1, track_misses: 0,
  },
];

/** frame_id=1, a DIFFERENT scenario ("simulated:05_multiple_obstacles" vs. the 08_* frames
 * above), reusing "track-1" -- every fresh `scripts/serve_unity_bridge.py` run's `ObjectTracker`
 * really does start numbering from "track-1" again (Phase 7), so this models the exact scenario-
 * switch scenario found live: a new producer whose own track-1 has nothing to do with the
 * previous producer's track-1. Also carries a second object ("track-9") the 08_* frames never
 * had, and deliberately omits any object with the SAME id as 08_*'s only other would-be track --
 * used to prove old, no-longer-relevant tracks don't linger. A genuinely different `session_id`
 * too (see docs/architecture.md "Session and sequence management") -- a real scenario switch
 * always mints a new one. */
export const REAL_SCENARIO_SWITCH_FRAME_MESSAGE: LiveMessage = {
  type: "frame",
  frame_id: 1,
  data: {
    session_id: "test-session-05",
    timestamp: 1786700000.0,
    scan_id: "a1b2c3d4-0000-0000-0000-000000000001",
    sequence_number: 1,
    source_id: "simulated:05_multiple_obstacles",
    objects: SCENARIO_SWITCH_OBJECTS,
    tracked_objects: deriveTrackedObjects(SCENARIO_SWITCH_OBJECTS, [], 1786700000.0, 1),
    events: [
      // A real session boundary: source_id AND session_id both changed -- see
      // docs/architecture.md "Session and sequence management".
      { event_type: "track_created", sequence_number: 1, timestamp: 1786700000.0, track_id: "track-1", previous_value: null, new_value: "confirmed", summary: "Track track-1 created (wall)." },
      { event_type: "track_created", sequence_number: 1, timestamp: 1786700000.0, track_id: "track-9", previous_value: null, new_value: "confirmed", summary: "Track track-9 created (pole_like)." },
    ],
    risk: {
      overall_risk: "safe",
      most_critical: null,
      results: [],
    },
    clearance: {
      front: { direction: "front", distance_m: 5.23, nearest_point: { x: 8.48, y: -8.48 } },
      rear: { direction: "rear", distance_m: 5.74, nearest_point: { x: -8.49, y: 8.49 } },
      left: { direction: "left", distance_m: 7.28, nearest_point: { x: 8.48, y: 8.48 } },
      right: { direction: "right", distance_m: 7.29, nearest_point: { x: -8.49, y: -8.49 } },
      min_clearance_m: 5.23,
      min_direction: "front",
      corridor_width_m: 16.97,
      overall_status: "safe",
      reason: ["Closest clearance is 5.23m, front."],
    },
    vehicle: { x: 0.0, y: 0.0, heading: 0.0, speed_mps: 0.0 },
    config: REAL_CRITICAL_FRAME_MESSAGE.data.config,
    map: null,
    points: null,
  },
};

/**
 * A REAL hardware-mode (`DATA_SOURCE=hardware`, `ESP32_TRANSPORT=mock`) wire frame -- generated
 * by driving an actual `STM32ProcessedFrame` through `MockESP32Transport -> ESP32Source ->
 * ProcessedFrameToLiveState -> build_perception_frame_message` (Phase 3 path), not hand-typed.
 * `source_id` is `stm32_hardware`, `config.data_source` is `"hardware"`, and `sensor_source` is
 * `"lidar+radar"` (real fusion attribution). Objects carry no width/depth/shape (the STM32 does
 * not transmit geometry) -- exactly as the Edge sends it.
 */
export const REAL_HARDWARE_FRAME_MESSAGE: LiveMessage = {
  type: "frame",
  frame_id: 0,
  broadcast_at: 1786602600.1,
  data: {
    session_id: "hw-session-1",
    timestamp: 1786602600.0,
    scan_id: "esp32-0",
    sequence_number: 0,
    source_id: "stm32_hardware",
    objects: [
      {
        track_id: "stm32-track-1", classification: "vehicle_like", confidence: 0.9,
        centroid: { x: 6.0, y: 0.0 }, width: 0.0, depth: 0.0, distance: 6.0,
        velocity: { vx: -2.0, vy: 0.0 }, direction: null, predicted_position: null,
        tracking_state: null, movement_state: null, track_age: null, track_hits: null,
        track_misses: null, point_count: null, aspect_ratio: null,
      },
    ],
    tracked_objects: [
      {
        track_id: "stm32-track-1", classification: "vehicle_like", confidence: 0.9,
        x: 6.0, y: 0.0, distance: 6.0, velocity: { vx: -2.0, vy: 0.0 },
        sensor_source: "lidar+radar", first_seen: 1786602600.0, last_seen: 1786602600.0,
        frames_tracked: null,
        trajectory: [
          { frame_id: 0, timestamp: 1786602600.0, x: 6.0, y: 0.0, vx: -2.0, vy: 0.0, distance: 6.0, classification: "vehicle_like", tracking_state: null },
        ],
        ttc: 3.0, risk: "warning", tracking_state: null, movement_state: null,
      },
    ],
    risk: {
      overall_risk: "warning",
      most_critical: {
        track_id: "stm32-track-1", classification: "vehicle_like", distance: 6.0, relative_speed: 2.0,
        in_projected_path: true, ttc: 3.0, collision_predicted: false, predicted_collision_time: null,
        predicted_collision_position: null, risk_level: "warning", risk_score: null,
        reason: ["scripted approaching vehicle at 6.0 m"],
      },
      results: [
        {
          track_id: "stm32-track-1", classification: "vehicle_like", distance: 6.0, relative_speed: 2.0,
          in_projected_path: true, ttc: 3.0, collision_predicted: false, predicted_collision_time: null,
          predicted_collision_position: null, risk_level: "warning", risk_score: null,
          reason: ["scripted approaching vehicle at 6.0 m"],
        },
      ],
    },
    clearance: {
      front: { direction: "front", distance_m: 3.5, nearest_point: null },
      rear: { direction: "rear", distance_m: 8.0, nearest_point: null },
      left: { direction: "left", distance_m: 3.0, nearest_point: null },
      right: { direction: "right", distance_m: 3.0, nearest_point: null },
      min_clearance_m: 3.0, min_direction: "left", corridor_width_m: 6.0,
      overall_status: "safe", reason: ["Directional clearance from STM32 processed frame."],
    },
    vehicle: { x: 0.0, y: 0.0, heading: 0.0, speed_mps: 0.0 },
    config: {
      vehicle_length_m: 4.5, vehicle_width_m: 1.8, front_safety_margin_m: 1.0, rear_safety_margin_m: 0.5,
      left_safety_margin_m: 0.3, right_safety_margin_m: 0.3, collision_warning_distance_m: 5.0,
      collision_critical_distance_m: 2.0, collision_warning_ttc_s: 4.0, collision_critical_ttc_s: 2.0,
      lidar_range_max_m: 12.0, data_source: "hardware",
    },
    sensor_status: {
      lidar: { connected: true, point_count: null, valid_percentage: null, mean_distance_m: null },
      radar: { connected: true, point_count: null, valid_percentage: null, mean_distance_m: null },
    },
    performance_metrics: { pipeline_processing_ms: 5.0, measured_scan_interval_s: null, measured_scan_rate_hz: null, scans_processed: 1 },
    events: [
      { event_type: "track_created", sequence_number: 0, timestamp: 1786602600.0, track_id: "stm32-track-1", previous_value: null, new_value: "unknown", summary: "Track stm32-track-1 created (vehicle_like)." },
    ],
    map: null,
    points: null,
  },
};

/** `GET /debug/stream-status` while hardware mode has NO data -- the ESP32 edge loop published a
 * HARDWARE_DATA_UNAVAILABLE ERROR, which `backend.ingestion` recorded on the connection. */
export const HARDWARE_UNAVAILABLE_STREAM_STATUS: StreamStatus = {
  connected: true, last_frame_id: null, last_frame_timestamp: null, frames_received: 0, frames_dropped: 0,
  source_id: "stm32_hardware", age_ms: null, risk: null, object_count: 0, track_count: 0,
  session_id: "hw-session-2", last_sequence: null, last_timestamp: null, frame_age_ms: null,
  scan_rate_hz: null, measured_scan_rate_hz: null, configured_scan_rate_hz: null, objects: 0, tracks: 0,
  backend_status: "ok", edge_status: "connected", websocket_status: "connected", dashboard_clients_connected: 1,
  latency_ms: null, sensor_ingestion_latency_ms: null,
  last_error_code: "HARDWARE_DATA_UNAVAILABLE",
  last_error_message: "stale data: newest processed frame is 4.2s old",
  last_error_age_ms: 1200,
};
