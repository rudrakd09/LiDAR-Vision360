/**
 * TypeScript interfaces mirroring the backend's wire schema exactly -- the same
 * `PERCEPTION_FRAME` `data` shape `perception/src/serialization/unity_protocol.py` builds and
 * docs/communication.md documents, and the same one `PerceptionDataTypes.cs` parses on the Unity
 * side. Three independent language implementations of one documented schema (Python producer,
 * C# consumer, TypeScript consumer) -- not a fourth invented shape.
 */

export interface Vec2 {
  x: number;
  y: number;
}

export interface Velocity {
  vx: number;
  vy: number;
}

export interface PerceptionObject {
  track_id: string;
  classification: "wall" | "vehicle_like" | "pole_like" | "person_like" | "large_obstacle" | "unknown";
  confidence: number;
  centroid: Vec2;
  width: number;
  depth: number;
  distance: number;
  velocity: Velocity | null;
  direction: number | null;
  predicted_position: Vec2 | null;
  tracking_state: "tentative" | "confirmed" | "coasting" | "lost" | null;
  movement_state: "stationary" | "moving" | "unknown" | null;
  track_age: number | null;
  track_hits: number | null;
  track_misses: number | null;
}

export interface CollisionResult {
  track_id: string | null;
  classification: string;
  distance: number;
  relative_speed: number;
  in_projected_path: boolean;
  ttc: number | null;
  collision_predicted: boolean;
  predicted_collision_time: number | null;
  predicted_collision_position: Vec2 | null;
  risk_level: "safe" | "warning" | "critical";
  risk_score: number | null;
  reason: string[];
}

export interface RiskData {
  overall_risk: "safe" | "warning" | "critical";
  most_critical: CollisionResult | null;
  results: CollisionResult[];
}

export type ClearanceDirection = "front" | "rear" | "left" | "right";
export type ClearanceStatus = "safe" | "caution" | "low_clearance" | "critical";

export interface DirectionalClearance {
  direction: ClearanceDirection;
  distance_m: number;
  nearest_point: Vec2 | null;
}

export interface ClearanceData {
  front: DirectionalClearance;
  rear: DirectionalClearance;
  left: DirectionalClearance;
  right: DirectionalClearance;
  min_clearance_m: number;
  min_direction: ClearanceDirection;
  corridor_width_m: number;
  overall_status: ClearanceStatus;
  reason: string[];
}

export interface VehicleData {
  x: number;
  y: number;
  heading: number;
  speed_mps: number;
}

export interface ConfigData {
  vehicle_length_m: number;
  vehicle_width_m: number;
  front_safety_margin_m: number;
  rear_safety_margin_m: number;
  left_safety_margin_m: number;
  right_safety_margin_m: number;
  collision_warning_distance_m: number;
  collision_critical_distance_m: number;
  collision_warning_ttc_s: number;
  collision_critical_ttc_s: number;
  lidar_range_max_m: number;
}

export interface OccupancyMapData {
  width_cells: number;
  height_cells: number;
  resolution_m: number;
  origin_x_m: number;
  origin_y_m: number;
  cells_base64: string;
}

export interface RawPoint {
  angle: number;
  distance: number;
  valid: boolean;
}

export interface PerceptionFrameData {
  timestamp: number;
  scan_id: string;
  sequence_number: number;
  source_id: string;
  objects: PerceptionObject[];
  risk: RiskData | null;
  clearance: ClearanceData | null;
  vehicle: VehicleData;
  config: ConfigData;
  map: OccupancyMapData | null;
  points: RawPoint[] | null;
}

export interface ConnectionStatus {
  state: "disconnected" | "connecting" | "connected" | "reconnecting";
  host: string;
  port: number;
  connected_at: number | null;
  last_message_at: number | null;
  frames_received: number;
  duplicate_or_out_of_order_dropped: number;
  last_frame_id: number | null;
  source_id: string | null;
  scan_rate_hz: number | null;
  dashboard_clients_connected: number;
}

export interface TrackRosterEntry extends PerceptionObject {
  seconds_since_seen: number;
}

export interface CombinedEvent {
  event_type: "collision" | "clearance";
  recorded_at: number;
  frame_id: number | null;
  timestamp: number;
  summary: string;
  detail: Record<string, unknown>;
}

export interface SessionRecord {
  id: string;
  source_id: string | null;
  scan_rate_hz: number | null;
  started_at: number;
  ended_at: number | null;
  frame_count: number;
  status: "active" | "ended";
}

/** `/ws/live` push messages -- see `backend.ingestion.PerceptionIngestor._dispatch`. */
export type LiveMessage =
  | { type: "snapshot"; data: { connection: ConnectionStatus; latest_frame: PerceptionFrameData | null } }
  | { type: "frame"; frame_id: number; data: PerceptionFrameData }
  | { type: "heartbeat"; data: { uptime_s: number; frames_sent: number; clients_connected: number } }
  | { type: "status"; data: { status: string; source_id: string | null; scan_rate_hz: number | null } }
  | { type: "error"; data: { code: string; message: string } };
