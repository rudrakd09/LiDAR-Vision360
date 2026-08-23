using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

// Mirrors perception/src/streaming/protocol.py (the envelope) and
// perception/src/serialization/unity_protocol.py (PERCEPTION_FRAME's `data` payload) exactly on
// the Python side -- keep all three in sync. See docs/communication.md "Protocol" for the
// authoritative schema reference and versioning policy. Uses Newtonsoft.Json (Unity package
// com.unity.nuget.newtonsoft-json, see docs/unity.md "Setup" for the install step) rather than
// Unity's built-in JsonUtility, which cannot represent nullable/optional fields (this schema has
// many -- velocity/direction/predicted_position/ttc/risk/map/points can each be legitimately
// absent), top-level arrays without an extra wrapper object, or -- needed here since Phase 12 --
// a polymorphic `data` field whose shape depends on `message_type` (`JObject`, deserialized a
// second time into the specific type once `message_type` is known -- see PerceptionTCPClient.cs).

/// <summary>The outer message envelope every Python -> Unity JSON message shares, regardless of
/// `messageType` -- see docs/communication.md "Message types". `data`'s actual shape depends on
/// `messageType`; it is parsed generically here (Newtonsoft's `JObject`) and re-parsed into the
/// specific matching type (`PerceptionFrameData`/`HeartbeatData`/`SystemStatusData`/`ErrorData`)
/// by <see cref="PerceptionTCPClient"/> once `messageType` is known.</summary>
[Serializable]
public class MessageEnvelope
{
    [JsonProperty("protocol_version")] public string protocolVersion;
    [JsonProperty("message_type")] public string messageType; // "PERCEPTION_FRAME" | "HEARTBEAT" | "SYSTEM_STATUS" | "ERROR"
    // Added for session/sequence management (see docs/architecture.md "Session and sequence
    // management") -- minted once per `scripts/serve_unity_bridge.py` run
    // (`pipeline.LiveStateBuilder.session_id`), carried on EVERY message type from that run, not
    // just PERCEPTION_FRAME. `null` for a producer/message that predates this field -- see
    // SessionValidator, which degrades to frame_id-only validation in that case, same as before.
    [JsonProperty("session_id")] public string sessionId;
    [JsonProperty("source_id")] public string sourceId;
    [JsonProperty("frame_id")] public long? frameId;            // null for non-PERCEPTION_FRAME message types
    [JsonProperty("timestamp")] public double timestamp;
    [JsonProperty("transmission_timestamp")] public double transmissionTimestamp;
    [JsonProperty("data")] public JObject data;
}

[Serializable]
public class Vec2Data
{
    [JsonProperty("x")] public float x;
    [JsonProperty("y")] public float y;
}

[Serializable]
public class VelocityData
{
    [JsonProperty("vx")] public float vx;
    [JsonProperty("vy")] public float vy;
}

[Serializable]
public class PerceptionObjectData
{
    [JsonProperty("track_id")] public string trackId;
    [JsonProperty("classification")] public string classification; // "wall" | "vehicle_like" | "pole_like" | "person_like" | "large_obstacle" | "unknown"
    [JsonProperty("confidence")] public float confidence;
    [JsonProperty("centroid")] public Vec2Data centroid;
    [JsonProperty("width")] public float width;
    [JsonProperty("depth")] public float depth;
    [JsonProperty("distance")] public float distance;
    [JsonProperty("velocity")] public VelocityData velocity;                 // null until tracking reports a reliable estimate
    [JsonProperty("direction")] public float? direction;                      // null under the same condition as velocity
    [JsonProperty("predicted_position")] public Vec2Data predictedPosition;   // null under the same condition
    [JsonProperty("tracking_state")] public string trackingState;             // "tentative" | "confirmed" | "coasting" | "lost"
    [JsonProperty("movement_state")] public string movementState;             // "stationary" | "moving" | "unknown"
    [JsonProperty("track_age")] public int? trackAge;
    [JsonProperty("track_hits")] public int? trackHits;
    [JsonProperty("track_misses")] public int? trackMisses;
}

/// <summary>One trajectory sample -- copied verbatim from the scan it was observed, never
/// re-derived/interpolated. See `models.live_state.TrajectoryPoint` on the Python side.</summary>
[Serializable]
public class TrajectoryPointData
{
    [JsonProperty("frame_id")] public long frameId;
    [JsonProperty("timestamp")] public double timestamp;
    [JsonProperty("x")] public float x;
    [JsonProperty("y")] public float y;
    [JsonProperty("vx")] public float? vx;
    [JsonProperty("vy")] public float? vy;
    [JsonProperty("distance")] public float distance;
    [JsonProperty("classification")] public string classification;
    [JsonProperty("tracking_state")] public string trackingState;
}

/// <summary>One tracked object's FULL Edge-computed view -- see `models.live_state.
/// TrackedObjectState` on the Python side. This is what any Unity script displaying per-track
/// detail (classification/TTC/risk/trajectory/sensor source) must read directly -- never
/// re-joined or re-derived here (see docs/architecture.md "Dashboard and Unity as pure LiveState
/// consumers"). Distinct from <see cref="PerceptionObjectData"/> (this scan's raw object, no
/// history) -- both arrive on every `PERCEPTION_FRAME`.</summary>
[Serializable]
public class TrackedObjectData
{
    [JsonProperty("track_id")] public string trackId;
    [JsonProperty("classification")] public string classification;
    [JsonProperty("confidence")] public float confidence;
    [JsonProperty("x")] public float x;
    [JsonProperty("y")] public float y;
    [JsonProperty("distance")] public float distance;
    [JsonProperty("velocity")] public VelocityData velocity;
    // Always "lidar" in this project's current 2D-LiDAR-only scope -- a real, structurally-true
    // label (exactly one sensor modality exists), not a fabricated measurement.
    [JsonProperty("sensor_source")] public string sensorSource;
    [JsonProperty("first_seen")] public double? firstSeen;
    [JsonProperty("last_seen")] public double? lastSeen;
    [JsonProperty("frames_tracked")] public int? framesTracked;
    [JsonProperty("trajectory")] public List<TrajectoryPointData> trajectory;
    [JsonProperty("ttc")] public float? ttc;
    [JsonProperty("risk")] public string risk; // "safe" | "warning" | "critical" | null
    [JsonProperty("tracking_state")] public string trackingState;
    [JsonProperty("movement_state")] public string movementState;
}

/// <summary>One collision-risk/clearance/track-lifecycle transition recorded THIS scan -- see
/// `models.live_state.LiveStateEvent`/`serialization.unity_protocol.build_events_payload`'s own
/// docstring (only this scan's own new events are sent per-frame, not the full retained backlog --
/// a consumer wanting a running timeline accumulates these itself, same pattern the dashboard's
/// own `hooks/useLiveEvents.ts` uses).</summary>
[Serializable]
public class LiveEventData
{
    [JsonProperty("event_type")] public string eventType; // "collision" | "clearance" | "track_created" | "track_lost" | "tracking_state_changed" | "ttc_change"
    [JsonProperty("sequence_number")] public long sequenceNumber;
    [JsonProperty("timestamp")] public double timestamp;
    [JsonProperty("track_id")] public string trackId;
    [JsonProperty("previous_value")] public string previousValue;
    [JsonProperty("new_value")] public string newValue;
    [JsonProperty("summary")] public string summary;
}

[Serializable]
public class CollisionResultData
{
    [JsonProperty("track_id")] public string trackId;
    [JsonProperty("classification")] public string classification;
    [JsonProperty("distance")] public float distance;
    [JsonProperty("relative_speed")] public float relativeSpeed;
    [JsonProperty("in_projected_path")] public bool inProjectedPath;
    [JsonProperty("ttc")] public float? ttc;                                  // null = not approaching / undefined, NOT "infinite" as a number
    [JsonProperty("collision_predicted")] public bool collisionPredicted;
    [JsonProperty("predicted_collision_time")] public float? predictedCollisionTime;
    [JsonProperty("predicted_collision_position")] public Vec2Data predictedCollisionPosition;
    [JsonProperty("risk_level")] public string riskLevel;                     // "safe" | "warning" | "critical"
    [JsonProperty("risk_score")] public float? riskScore;
    [JsonProperty("reason")] public List<string> reason;
}

[Serializable]
public class RiskData
{
    [JsonProperty("overall_risk")] public string overallRisk;
    [JsonProperty("most_critical")] public CollisionResultData mostCritical;  // null if no tracked objects this scan
    [JsonProperty("results")] public List<CollisionResultData> results;
}

[Serializable]
public class OccupancyMapData
{
    [JsonProperty("width_cells")] public int widthCells;
    [JsonProperty("height_cells")] public int heightCells;
    [JsonProperty("resolution_m")] public float resolutionM;
    [JsonProperty("origin_x_m")] public float originXM;
    [JsonProperty("origin_y_m")] public float originYM;
    [JsonProperty("cells_base64")] public string cellsBase64; // row-major, 1 byte/cell: 0=UNKNOWN, 1=FREE, 2=OCCUPIED
}

[Serializable]
public class VehicleData
{
    [JsonProperty("x")] public float x;
    [JsonProperty("y")] public float y;
    [JsonProperty("heading")] public float heading;
    [JsonProperty("speed_mps")] public float speedMps;
}

[Serializable]
public class RawPointData
{
    [JsonProperty("angle")] public float angle;
    [JsonProperty("distance")] public float distance;
    [JsonProperty("valid")] public bool valid;
}

/// <summary>One direction's clearance reading -- the `data.clearance.front/rear/left/right` shape
/// `perception/src/serialization/unity_protocol.py`'s `build_clearance_payload` produces (Phase
/// 10). `nearestPoint` is `null` when no LiDAR return was present in that direction's quadrant
/// (`distanceM` still reflects the sensor-range cap in that case -- see
/// `models.clearance.DirectionalClearance`'s own docstring on the Python side).</summary>
[Serializable]
public class DirectionalClearanceData
{
    [JsonProperty("direction")] public string direction; // "front" | "rear" | "left" | "right"
    [JsonProperty("distance_m")] public float distanceM;
    [JsonProperty("nearest_point")] public Vec2Data nearestPoint;
}

/// <summary>The `data.clearance` payload of a `PERCEPTION_FRAME` message (Phase 10) -- see
/// `perception/src/serialization/unity_protocol.py` (`build_clearance_payload`) for the producing
/// side and docs/collision.md "Directional clearance" for the schema reference. Replaces the
/// earlier untyped `object clearance` placeholder now that the engine producing this field
/// actually exists -- see `PerceptionFrameData.clearance`'s own remarks.</summary>
[Serializable]
public class ClearanceData
{
    [JsonProperty("front")] public DirectionalClearanceData front;
    [JsonProperty("rear")] public DirectionalClearanceData rear;
    [JsonProperty("left")] public DirectionalClearanceData left;
    [JsonProperty("right")] public DirectionalClearanceData right;
    [JsonProperty("min_clearance_m")] public float minClearanceM;
    [JsonProperty("min_direction")] public string minDirection; // "front" | "rear" | "left" | "right"
    [JsonProperty("corridor_width_m")] public float corridorWidthM;
    [JsonProperty("overall_status")] public string overallStatus; // "safe" | "caution" | "low_clearance" | "critical"
    [JsonProperty("reason")] public List<string> reason;
}

/// <summary>Vehicle geometry and risk thresholds -- sent every frame so SafetyZoneRenderer and
/// CollisionRiskIndicator never hard-code a threshold themselves (see docs/unity.md "Safety
/// zones": "Python is the authority for risk calculation. Unity only visualizes the
/// result.").</summary>
[Serializable]
public class ConfigData
{
    [JsonProperty("vehicle_length_m")] public float vehicleLengthM;
    [JsonProperty("vehicle_width_m")] public float vehicleWidthM;
    [JsonProperty("front_safety_margin_m")] public float frontSafetyMarginM;
    [JsonProperty("rear_safety_margin_m")] public float rearSafetyMarginM;
    [JsonProperty("left_safety_margin_m")] public float leftSafetyMarginM;
    [JsonProperty("right_safety_margin_m")] public float rightSafetyMarginM;
    [JsonProperty("collision_warning_distance_m")] public float collisionWarningDistanceM;
    [JsonProperty("collision_critical_distance_m")] public float collisionCriticalDistanceM;
    [JsonProperty("collision_warning_ttc_s")] public float collisionWarningTtcS;
    [JsonProperty("collision_critical_ttc_s")] public float collisionCriticalTtcS;
    [JsonProperty("lidar_range_max_m")] public float lidarRangeMaxM;
    // "simulation" | "hardware" -- Settings.data_source (see docs/architecture.md "Sensor source
    // abstraction" / "Session and sequence management"). Null for an older payload that predates
    // this field.
    [JsonProperty("data_source")] public string dataSource;
}

/// <summary>The `data` payload of a `PERCEPTION_FRAME` message (i.e. `envelope.data` once
/// re-parsed) -- see `perception/src/serialization/unity_protocol.py` (`build_frame_message`)
/// for the producing side and docs/communication.md "Perception frame" for the schema reference.
/// No `protocol_version` field here -- that lives on <see cref="MessageEnvelope"/> only, not
/// duplicated at this level (a Phase 11 -> Phase 12 change, see that module's own docstring on
/// the Python side).</summary>
[Serializable]
public class PerceptionFrameData
{
    // See docs/architecture.md "Session and sequence management" -- minted once per
    // `scripts/serve_unity_bridge.py` run, same value the envelope itself now also carries
    // (MessageEnvelope.sessionId). Null for an older payload that predates this field.
    [JsonProperty("session_id")] public string sessionId;
    [JsonProperty("timestamp")] public double timestamp;
    [JsonProperty("scan_id")] public string scanId;
    [JsonProperty("sequence_number")] public long sequenceNumber;
    [JsonProperty("source_id")] public string sourceId;
    [JsonProperty("objects")] public List<PerceptionObjectData> objects;
    // The richer, history-joined per-track view -- see TrackedObjectData's own docstring. Null
    // for an older payload that predates this field; the live backend always sends it (as an
    // empty list when nothing is currently tracked, never null for "nothing tracked").
    [JsonProperty("tracked_objects")] public List<TrackedObjectData> trackedObjects;
    // Transitions recorded THIS scan only -- see LiveEventData's own docstring.
    [JsonProperty("events")] public List<LiveEventData> events;
    [JsonProperty("risk")] public RiskData risk;                 // null if the collision stage wasn't wired into this bridge run
    [JsonProperty("clearance")] public ClearanceData clearance;   // null if the clearance stage wasn't wired into this bridge run (see docs/collision.md "Directional clearance" -- Phase 10, implemented)
    [JsonProperty("vehicle")] public VehicleData vehicle;
    [JsonProperty("config")] public ConfigData config;              // vehicle geometry + risk thresholds -- always present, see ConfigData
    [JsonProperty("map")] public OccupancyMapData map;             // null on scans that don't include a map update (see scripts/serve_unity_bridge.py --map-every-n-scans)
    [JsonProperty("points")] public List<RawPointData> points;     // null unless explicitly requested (see scripts/serve_unity_bridge.py --include-points, off by default)
    // Real, per-scan-measured performance figures (see docs/architecture.md "Session and
    // sequence management", models.live_state.PerformanceMetrics) -- null for an older payload
    // that predates this field.
    [JsonProperty("performance_metrics")] public PerformanceMetricsData performanceMetrics;
}

/// <summary>Real, directly-measured performance figures for this scan -- never a fabricated
/// throughput/rate figure this process doesn't actually measure. See
/// `models.live_state.PerformanceMetrics`'s own docstring on the Python side.</summary>
[Serializable]
public class PerformanceMetricsData
{
    [JsonProperty("pipeline_processing_ms")] public float? pipelineProcessingMs;
    [JsonProperty("measured_scan_interval_s")] public float? measuredScanIntervalS;
    [JsonProperty("measured_scan_rate_hz")] public float? measuredScanRateHz;
    [JsonProperty("scans_processed")] public int scansProcessed;
}

/// <summary>The `data` payload of a `HEARTBEAT` message -- sent on `streaming_heartbeat_interval_s`
/// regardless of whether a real `PERCEPTION_FRAME` was also published in that window, so a
/// client can distinguish "no new scan yet" from "the server has stopped talking to me." See
/// docs/communication.md "Heartbeat / connection status".</summary>
[Serializable]
public class HeartbeatData
{
    [JsonProperty("uptime_s")] public float uptimeS;
    [JsonProperty("frames_sent")] public int framesSent;
    [JsonProperty("clients_connected")] public int clientsConnected;
}

/// <summary>The `data` payload of a `SYSTEM_STATUS` message -- sent once per connection (not on
/// a repeating interval), static-ish session information.</summary>
[Serializable]
public class SystemStatusData
{
    [JsonProperty("status")] public string status;
    [JsonProperty("source_id")] public string sourceId;
    [JsonProperty("scan_rate_hz")] public float? scanRateHz;
}

/// <summary>The `data` payload of an `ERROR` message -- sent when the server catches an internal
/// error it recovered from (e.g. one scan's pipeline run raised, but the server itself keeps
/// running); lets a connected client surface *something* on the HUD instead of the frame stream
/// just silently stalling.</summary>
[Serializable]
public class ErrorData
{
    [JsonProperty("code")] public string code;
    [JsonProperty("message")] public string message;
}
