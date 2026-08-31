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
  /** Real geometric features the classifier already computed (`objects.features.extract_features`
   * / `DetectedObject.point_count`/`shape_features.aspect_ratio`) -- `null` when the source object
   * genuinely didn't have them, and optional (`?`) rather than required so older captured/test
   * payloads (from before these two fields were added to the wire protocol) still type-check
   * without being rewritten -- the live backend always sends the key, older fixtures just predate
   * it. */
  point_count?: number | null;
  aspect_ratio?: number | null;
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
  /** `"simulation"` | `"hardware"` -- `Settings.data_source`, see docs/architecture.md "Sensor
   * source abstraction". Optional for the same backward-compat reasoning as everywhere else in
   * this file (older captured payloads predate this field). */
  data_source?: "simulation" | "hardware" | null;
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

/** Per-modality real sensor status this scan -- see `models.live_state.SensorChannelStatus`. A
 * modality this deployment has no sensor for at all (e.g. `radar`, in this project's current
 * 2D-LiDAR-only scope) is `null`, never a fabricated reading. */
export interface SensorChannelStatus {
  connected: boolean;
  point_count: number | null;
  valid_percentage: number | null;
  mean_distance_m: number | null;
}

/** Real, directly-measured performance figures for this scan -- see
 * `models.live_state.PerformanceMetrics`. `measured_scan_rate_hz` is the Edge's own per-scan
 * measurement (real inter-scan wall-clock interval), the authoritative source for scan rate --
 * prefer this over any client-side-measured rate (see `hooks/useMeasuredScanRate.ts`'s own
 * updated docstring). */
export interface PerformanceMetrics {
  pipeline_processing_ms: number | null;
  measured_scan_interval_s: number | null;
  measured_scan_rate_hz: number | null;
  scans_processed: number;
}

/** One trajectory sample -- copied verbatim from the scan it was observed, never
 * re-derived/interpolated. See `models.live_state.TrajectoryPoint`. */
export interface TrajectoryPoint {
  frame_id: number;
  timestamp: number;
  x: number;
  y: number;
  vx: number | null;
  vy: number | null;
  distance: number;
  classification: string;
  tracking_state: string | null;
}

/** One tracked object's FULL Edge-computed view -- see `models.live_state.TrackedObjectState`.
 * This is what `DETECTED OBJECTS`/`TRACKING` sections must render directly: `ttc`/`risk` are
 * already joined by track_id at the Edge (`pipeline.LiveStateBuilder`), `first_seen`/`last_seen`/
 * `frames_tracked`/`trajectory` already come from the real tracker's own history
 * (`tracking.TrackHistory`) -- the dashboard must never re-join or re-derive any of these itself
 * (see docs/architecture.md "Dashboard and Unity as pure LiveState consumers"). */
export interface TrackedObjectData {
  track_id: string;
  classification: PerceptionObject["classification"];
  confidence: number;
  x: number;
  y: number;
  distance: number;
  velocity: Velocity | null;
  /** Always `"lidar"` in this project's current 2D-LiDAR-only scope -- a real, structurally-true
   * label (exactly one sensor modality exists), not a fabricated measurement. Will reflect
   * additional modalities once/if this project ever integrates one. */
  sensor_source: string;
  first_seen: number | null;
  last_seen: number | null;
  frames_tracked: number | null;
  trajectory: TrajectoryPoint[];
  ttc: number | null;
  risk: "safe" | "warning" | "critical" | null;
  tracking_state: "tentative" | "confirmed" | "coasting" | "lost" | null;
  movement_state: "stationary" | "moving" | "unknown" | null;
}

/** One collision-risk/clearance/track-lifecycle transition, recorded by `pipeline.
 * LiveStateBuilder` the scan it happened -- see `models.live_state.LiveStateEvent`. Only events
 * recorded on THIS scan are sent per-frame (see `serialization.unity_protocol.
 * build_events_payload`'s own docstring for why) -- `hooks/useLiveEvents.ts` accumulates them
 * into the running Event Timeline client-side (buffering a server-told stream, not calculating
 * anything -- same pattern this project already established for trajectories). */
export interface LiveEvent {
  /** `"connection"` entries are added client-side by `hooks/useConnectionEvents` from real
   * observed connection/freshness transitions (not from the wire); all others are the Edge's own
   * `LiveState.events`. */
  event_type: "collision" | "clearance" | "track_created" | "track_lost" | "tracking_state_changed" | "ttc_change" | "connection";
  sequence_number: number;
  timestamp: number;
  track_id: string | null;
  previous_value: string | null;
  new_value: string;
  summary: string;
}

export interface PerceptionFrameData {
  /** Minted once per Edge process run (see `pipeline.LiveStateBuilder`) -- the SAME session_id
   * every message from that run carries, at both the envelope level and here in `data`. This is
   * the identity `useLiveSocket`'s own session-boundary detection compares against to decide
   * "is this a new session I must reset all derived state for" -- see docs/architecture.md
   * "Session and sequence management". Optional/nullable, same reasoning as `point_count`/
   * `aspect_ratio` above -- older captured/test payloads (from before this field existed)
   * still type-check unchanged; the live backend always sends it. */
  session_id?: string | null;
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
  /** `null` for a payload built before these fields existed (older captured fixtures) -- the live
   * backend always sends them. */
  sensor_status?: Record<string, SensorChannelStatus | null> | null;
  performance_metrics?: PerformanceMetrics | null;
  /** The richer, history-joined per-track view -- see `TrackedObjectData`. `null` only for a
   * payload built before this field existed; the live backend always sends it (as `[]` when
   * nothing is currently tracked, never `null` for "nothing tracked"). */
  tracked_objects?: TrackedObjectData[] | null;
  /** Transitions recorded THIS scan only -- see `LiveEvent`'s own docstring. `[]` is the normal
   * case (no transition happened this scan); `null` only for a payload built before this field
   * existed. */
  events?: LiveEvent[] | null;
}

export type SessionStatus = "active" | "stale" | "disconnected";

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
  /** "active" | "stale" | "disconnected" -- server-computed (backend.state.compute_session_status),
   * distinct from `state` (the raw TCP state): a socket can stay "connected" while the bridge has
   * stalled without closing it. See docs/cloud.md "Session lifecycle". */
  session_status: SessionStatus;
  /** Edge-minted wire session_id currently being displayed -- see docs/architecture.md "Session
   * and sequence management". Optional, same backward-compat reasoning as `PerceptionFrameData.
   * session_id`. */
  session_id?: string | null;
  /** Resets to 0 on every new session boundary -- unlike `frames_received` above, which stays
   * cumulative for this backend process's whole lifetime. */
  session_frames_received?: number;
  /** Real, per-scan-measured (from the Edge's own LiveState.performance_metrics) -- vs.
   * `scan_rate_hz` above, which is the possibly-stale SYSTEM_STATUS configured-target value. */
  measured_scan_rate_hz?: number | null;
}

export interface TrackRosterEntry extends PerceptionObject {
  seconds_since_seen: number;
  first_seen_at: number | null;
  last_seen_at: number;
  frames_tracked: number;
}

/** GET /api/tracks/{track_id} -- see `backend.state.LatestState.track_summary`. */
export interface TrackSummary {
  track_id: string;
  current: PerceptionObject;
  first_seen_at: number | null;
  last_seen_at: number;
  seconds_since_seen: number;
  frames_tracked: number;
  history_length: number;
}

/** One point of GET /api/tracking-history?track_id=... -- see `LatestState._append_track_history`.
 * Every field here is copied verbatim from a real `objects[]` entry the perception tracker (Phase
 * 7) already produced; nothing re-derived. */
export interface TrackHistoryPoint {
  frame_id: number | null;
  timestamp: number | null;
  x: number | null;
  y: number | null;
  vx: number | null;
  vy: number | null;
  distance: number | null;
  classification: string | null;
  tracking_state: string | null;
}

export interface CombinedEvent {
  event_type: "collision" | "clearance" | "ttc" | "sensor";
  recorded_at: number;
  frame_id: number | null;
  timestamp: number;
  summary: string;
  detail: Record<string, unknown>;
}

/** GET /debug/stream-status -- see `backend.routes.debug`. Purely a diagnostic/demo cross-check
 * ("is the problem backend/WebSocket/frontend/perception") -- the dashboard's own live rendering
 * never depends on this; that's `/ws/live`'s job. */
export interface StreamStatus {
  connected: boolean;
  last_frame_id: number | null;
  last_frame_timestamp: number | null;
  frames_received: number;
  frames_dropped: number;
  source_id: string | null;
  age_ms: number | null;
  risk: string | null;
  object_count: number;
  track_count: number;
  /** See docs/architecture.md "Session and sequence management" / `backend.routes.debug`'s own
   * docstring for what each of these adds beyond the fields above. Optional for the same
   * backward-compat reasoning as everywhere else in this file. */
  session_id?: string | null;
  last_sequence?: number | null;
  last_timestamp?: number | null;
  frame_age_ms?: number | null;
  scan_rate_hz?: number | null;
  measured_scan_rate_hz?: number | null;
  configured_scan_rate_hz?: number | null;
  objects?: number;
  tracks?: number;
  backend_status?: string;
  edge_status?: string;
  websocket_status?: string;
  dashboard_clients_connected?: number;
  latency_ms?: number | null;
  /** Sensor capture -> backend receipt (includes Edge pipeline processing time) -- see
   * docs/architecture.md "Real-time performance monitoring". */
  sensor_ingestion_latency_ms?: number | null;
  /** Last producer ERROR message -- Phase 3 hardware mode sets `code =
   * "HARDWARE_DATA_UNAVAILABLE"` with a concrete `message` (ESP32 disconnected / timeout /
   * stale data / invalid frame / protocol error). Cleared automatically on the next real
   * perception frame. All null/absent in normal operation. */
  last_error_code?: string | null;
  last_error_message?: string | null;
  last_error_age_ms?: number | null;
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
  | { type: "frame"; frame_id: number; data: PerceptionFrameData; broadcast_at?: number }
  | { type: "heartbeat"; data: { uptime_s: number; frames_sent: number; clients_connected: number } }
  | { type: "status"; data: { status: string; source_id: string | null; scan_rate_hz: number | null } }
  | { type: "error"; data: { code: string; message: string } };
