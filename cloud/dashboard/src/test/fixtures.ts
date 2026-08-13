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
import type { LiveMessage } from "../types";

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
    },
    latest_frame: null,
  },
};

/** frame_id=13, real captured payload -- `risk.overall_risk` is already "critical" with a finite
 * `ttc`, one tracked object, and real (non-round) clearance numbers -- exactly the state the
 * dashboard was reported stuck at "safe/N-A/0" instead of showing. */
export const REAL_CRITICAL_FRAME_MESSAGE: LiveMessage = {
  type: "frame",
  frame_id: 13,
  data: {
    timestamp: 1786602527.0760803,
    scan_id: "17a71206-da65-413f-8fb4-8da8ec3e30e6",
    sequence_number: 13,
    source_id: "simulated:08_approaching_obstacle",
    objects: [
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
    ],
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
      results: [
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
      ],
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
    },
    map: null,
    points: null,
  },
};

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
    objects: [
      {
        ...REAL_CRITICAL_FRAME_MESSAGE.data.objects[0],
        centroid: { x: 4.9548, y: 0.0002 },
        distance: 4.9548,
        track_hits: 15,
        track_age: 15,
      },
    ],
    risk: {
      ...REAL_CRITICAL_FRAME_MESSAGE.data.risk!,
      most_critical: {
        ...REAL_CRITICAL_FRAME_MESSAGE.data.risk!.most_critical!,
        distance: 4.9548,
        ttc: 0.7763,
        predicted_collision_time: 0.8,
      },
    },
    clearance: {
      ...REAL_CRITICAL_FRAME_MESSAGE.data.clearance!,
      front: { direction: "front", distance_m: 1.6942, nearest_point: { x: 4.9442, y: -0.8718 } },
      min_clearance_m: 1.6942,
    },
  },
};
