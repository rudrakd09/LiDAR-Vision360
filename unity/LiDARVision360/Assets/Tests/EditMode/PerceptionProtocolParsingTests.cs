using Newtonsoft.Json;
using NUnit.Framework;

/// <summary>
/// Tests for parsing the Python -> Unity message envelope and payload types
/// (<c>PerceptionDataTypes.cs</c>) -- known-good messages, missing/null optional fields, and
/// malformed JSON. Mirrors the spirit of
/// `perception/tests/test_streaming_protocol.py`/`test_serialization_unity_protocol.py` on the
/// Python side, applied to the C# deserialization side of the same wire format.
///
/// <b>Written but not run in this environment</b> -- no Unity Editor was available (see
/// docs/unity.md "Status"). Run via Window &gt; General &gt; Test Runner &gt; EditMode once you
/// have the project open.
/// </summary>
public class PerceptionProtocolParsingTests
{
    const string ExamplePerceptionFrameEnvelope = @"{
        ""protocol_version"": ""2.0.0"",
        ""message_type"": ""PERCEPTION_FRAME"",
        ""frame_id"": 42,
        ""timestamp"": 1234567890.123,
        ""transmission_timestamp"": 1234567890.130,
        ""data"": {
            ""timestamp"": 1234567890.123,
            ""scan_id"": ""abc-123"",
            ""sequence_number"": 42,
            ""source_id"": ""simulated:08_approaching_obstacle"",
            ""objects"": [
                {
                    ""track_id"": ""track-3"",
                    ""classification"": ""vehicle_like"",
                    ""confidence"": 0.86,
                    ""centroid"": {""x"": 6.2, ""y"": 0.1},
                    ""width"": 1.8,
                    ""depth"": 4.2,
                    ""distance"": 6.2,
                    ""velocity"": {""vx"": -1.8, ""vy"": 0.0},
                    ""direction"": 182.5,
                    ""predicted_position"": {""x"": 6.0, ""y"": 0.1},
                    ""tracking_state"": ""confirmed"",
                    ""movement_state"": ""moving"",
                    ""track_age"": 12,
                    ""track_hits"": 12,
                    ""track_misses"": 0
                }
            ],
            ""risk"": {
                ""overall_risk"": ""warning"",
                ""most_critical"": null,
                ""results"": []
            },
            ""clearance"": null,
            ""vehicle"": {""x"": 0.0, ""y"": 0.0, ""heading"": 0.0, ""speed_mps"": 0.0},
            ""config"": {
                ""vehicle_length_m"": 4.5, ""vehicle_width_m"": 1.8,
                ""front_safety_margin_m"": 1.0, ""rear_safety_margin_m"": 0.5,
                ""left_safety_margin_m"": 0.3, ""right_safety_margin_m"": 0.3,
                ""collision_warning_distance_m"": 5.0, ""collision_critical_distance_m"": 2.0,
                ""collision_warning_ttc_s"": 4.0, ""collision_critical_ttc_s"": 2.0,
                ""lidar_range_max_m"": 12.0
            },
            ""map"": null,
            ""points"": null
        }
    }";

    [Test]
    public void ParsesEnvelopeTopLevelFields()
    {
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(ExamplePerceptionFrameEnvelope);
        Assert.AreEqual("2.0.0", envelope.protocolVersion);
        Assert.AreEqual("PERCEPTION_FRAME", envelope.messageType);
        Assert.AreEqual(42, envelope.frameId);
        Assert.IsNotNull(envelope.data);
    }

    [Test]
    public void ParsesNestedPerceptionFrameData()
    {
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(ExamplePerceptionFrameEnvelope);
        var frame = envelope.data.ToObject<PerceptionFrameData>();

        Assert.AreEqual("abc-123", frame.scanId);
        Assert.AreEqual(42, frame.sequenceNumber);
        Assert.AreEqual(1, frame.objects.Count);

        var obj = frame.objects[0];
        Assert.AreEqual("track-3", obj.trackId);
        Assert.AreEqual("vehicle_like", obj.classification);
        Assert.AreEqual(6.2f, obj.centroid.x, 1e-4f);
        Assert.IsNotNull(obj.velocity);
        Assert.AreEqual(-1.8f, obj.velocity.vx, 1e-4f);
        Assert.IsNotNull(obj.predictedPosition);
    }

    [Test]
    public void ParsesRiskBlock()
    {
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(ExamplePerceptionFrameEnvelope);
        var frame = envelope.data.ToObject<PerceptionFrameData>();
        Assert.AreEqual("warning", frame.risk.overallRisk);
        Assert.IsNull(frame.risk.mostCritical);
    }

    [Test]
    public void ParsesConfigBlock()
    {
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(ExamplePerceptionFrameEnvelope);
        var frame = envelope.data.ToObject<PerceptionFrameData>();
        Assert.AreEqual(4.5f, frame.config.vehicleLengthM, 1e-4f);
        Assert.AreEqual(2.0f, frame.config.collisionCriticalDistanceM, 1e-4f);
    }

    [Test]
    public void NullClearanceDoesNotThrow()
    {
        // Phase 10 (clearance engine) is implemented, but `clearance` stays nullable for a caller
        // running without that stage wired up -- must not throw or need special-casing either way.
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(ExamplePerceptionFrameEnvelope);
        var frame = envelope.data.ToObject<PerceptionFrameData>();
        Assert.IsNull(frame.clearance);
    }

    [Test]
    public void PopulatedClearanceParsesAllFourDirections()
    {
        const string json = @"{
            ""front"": {""direction"": ""front"", ""distance_m"": 4.08, ""nearest_point"": {""x"": 7.33, ""y"": 0.26}},
            ""rear"": {""direction"": ""rear"", ""distance_m"": 5.73, ""nearest_point"": null},
            ""left"": {""direction"": ""left"", ""distance_m"": 7.29, ""nearest_point"": {""x"": 8.49, ""y"": 8.49}},
            ""right"": {""direction"": ""right"", ""distance_m"": 7.29, ""nearest_point"": null},
            ""min_clearance_m"": 4.08, ""min_direction"": ""front"", ""corridor_width_m"": 16.97,
            ""overall_status"": ""safe"", ""reason"": [""Closest clearance is 4.08m, front.""]
        }";
        var clearance = JsonConvert.DeserializeObject<ClearanceData>(json);
        Assert.AreEqual("front", clearance.front.direction);
        Assert.AreEqual(4.08f, clearance.front.distanceM, 1e-4f);
        Assert.IsNotNull(clearance.front.nearestPoint);
        Assert.AreEqual(7.33f, clearance.front.nearestPoint.x, 1e-4f);
        Assert.IsNull(clearance.rear.nearestPoint);
        Assert.AreEqual("front", clearance.minDirection);
        Assert.AreEqual("safe", clearance.overallStatus);
        Assert.AreEqual(1, clearance.reason.Count);
    }

    [Test]
    public void NullMapAndPointsDoNotThrow()
    {
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(ExamplePerceptionFrameEnvelope);
        var frame = envelope.data.ToObject<PerceptionFrameData>();
        Assert.IsNull(frame.map);
        Assert.IsNull(frame.points);
    }

    [Test]
    public void MissingOptionalVelocityFieldsParseAsNull()
    {
        const string json = @"{
            ""track_id"": ""t1"", ""classification"": ""unknown"", ""confidence"": 0.0,
            ""centroid"": {""x"": 1.0, ""y"": 1.0}, ""width"": 0.5, ""depth"": 0.5, ""distance"": 1.4,
            ""velocity"": null, ""direction"": null, ""predicted_position"": null,
            ""tracking_state"": ""tentative"", ""movement_state"": ""unknown"",
            ""track_age"": 1, ""track_hits"": 1, ""track_misses"": 0
        }";
        var obj = JsonConvert.DeserializeObject<PerceptionObjectData>(json);
        Assert.IsNull(obj.velocity);
        Assert.IsNull(obj.direction);
        Assert.IsNull(obj.predictedPosition);
    }

    [Test]
    public void HeartbeatMessageParses()
    {
        const string json = @"{
            ""protocol_version"": ""2.0.0"", ""message_type"": ""HEARTBEAT"", ""frame_id"": null,
            ""timestamp"": 1000.0, ""transmission_timestamp"": 1000.0,
            ""data"": {""uptime_s"": 12.3, ""frames_sent"": 120, ""clients_connected"": 1}
        }";
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(json);
        Assert.AreEqual("HEARTBEAT", envelope.messageType);
        Assert.IsNull(envelope.frameId);

        var heartbeat = envelope.data.ToObject<HeartbeatData>();
        Assert.AreEqual(120, heartbeat.framesSent);
        Assert.AreEqual(1, heartbeat.clientsConnected);
    }

    [Test]
    public void ErrorMessageParses()
    {
        const string json = @"{
            ""protocol_version"": ""2.0.0"", ""message_type"": ""ERROR"", ""frame_id"": null,
            ""timestamp"": 1000.0, ""transmission_timestamp"": 1000.0,
            ""data"": {""code"": ""PIPELINE_ERROR"", ""message"": ""boom""}
        }";
        var envelope = JsonConvert.DeserializeObject<MessageEnvelope>(json);
        var error = envelope.data.ToObject<ErrorData>();
        Assert.AreEqual("PIPELINE_ERROR", error.code);
        Assert.AreEqual("boom", error.message);
    }

    [Test]
    public void MalformedJsonThrowsRatherThanSilentlyCorrupting()
    {
        // PerceptionTCPClient itself catches this exception and skips the line (see its own
        // TryParseEnvelope) -- this test documents the underlying Newtonsoft behavior that
        // design relies on: a malformed line must raise, not silently return a half-populated or
        // default-valued object that could be mistaken for real data.
        Assert.Throws<JsonReaderException>(() => JsonConvert.DeserializeObject<MessageEnvelope>("{not valid json"));
    }

    [Test]
    public void EmptyObjectsArrayParsesAsEmptyListNotNull()
    {
        const string json = @"{
            ""timestamp"": 1000.0, ""scan_id"": ""s"", ""sequence_number"": 0, ""source_id"": ""t"",
            ""objects"": [], ""risk"": null, ""clearance"": null,
            ""vehicle"": {""x"": 0.0, ""y"": 0.0, ""heading"": 0.0, ""speed_mps"": 0.0},
            ""config"": {
                ""vehicle_length_m"": 4.5, ""vehicle_width_m"": 1.8,
                ""front_safety_margin_m"": 1.0, ""rear_safety_margin_m"": 0.5,
                ""left_safety_margin_m"": 0.3, ""right_safety_margin_m"": 0.3,
                ""collision_warning_distance_m"": 5.0, ""collision_critical_distance_m"": 2.0,
                ""collision_warning_ttc_s"": 4.0, ""collision_critical_ttc_s"": 2.0,
                ""lidar_range_max_m"": 12.0
            },
            ""map"": null, ""points"": null
        }";
        var frame = JsonConvert.DeserializeObject<PerceptionFrameData>(json);
        Assert.IsNotNull(frame.objects);
        Assert.AreEqual(0, frame.objects.Count);
        Assert.IsNull(frame.risk); // collision stage not wired into this particular message -- must not throw
    }
}
