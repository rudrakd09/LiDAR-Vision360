using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using Newtonsoft.Json;
using UnityEngine;

/// <summary>
/// Connects to the structured JSON perception protocol server
/// (<c>perception.streaming.PerceptionStreamServer</c>, default <c>127.0.0.1:5006</c>) -- one
/// versioned envelope per line (newline-delimited JSON), self-delimiting unlike the legacy
/// <c>&lt;START&gt;</c>/<c>&lt;END&gt;</c> raw protocol, so no extra framing markers are needed
/// for this stream. See docs/communication.md "Protocol" / "Message framing".
///
/// Reads on a background thread -- the same pattern the existing, working
/// <c>LidarSerialReader.cs</c> already uses -- so a slow, stalled, or disconnected server
/// connection never blocks Unity's main thread. Only <c>Update()</c> (main thread) ever touches
/// Unity APIs or fires events, per this phase's explicit threading requirement (see
/// docs/communication.md "Unity main thread"). Automatically retries the connection on disconnect
/// (<see cref="reconnectDelaySeconds"/>) rather than giving up, and never opens a second
/// connection while one is already open/connecting -- see "Connection status".
///
/// <para><b>Frame ordering</b> (Phase 12): every `PERCEPTION_FRAME` carries a monotonically
/// increasing `frame_id` (the source `TrackedScan`'s own `sequence_number`). This client rejects
/// a duplicate or out-of-order `frame_id` before ever firing <see cref="OnPerceptionFrameReceived"/>
/// -- a real-time renderer must never let an older frame overwrite a newer one already applied
/// -- mirroring `streaming.protocol.classify_frame_id`'s exact logic on the Python side (see
/// docs/communication.md "Frame IDs"). A *gap* (one or more `frame_id`s skipped, but this one is
/// still newer) is accepted, not rejected: real-time visualization needs the current state, not
/// a guarantee every historical frame was seen.</para>
///
/// <para><b>Staleness</b> (Phase 12): <see cref="ConnectionState.Stale"/> is reported once
/// `connectionTimeoutSeconds` have passed with no message at all (frame or heartbeat) received,
/// even though the TCP socket itself may still be open -- distinct from
/// <see cref="ConnectionState.Disconnected"/> (the socket actually closed/errored).</para>
/// </summary>
public class PerceptionTCPClient : MonoBehaviour, ILidarDataSource
{
    [Header("Connection")]
    public string host = "127.0.0.1";
    public int port = 5006;
    [Tooltip("Seconds between reconnect attempts after a disconnect.")]
    public float reconnectDelaySeconds = 2f;
    [Tooltip("Seconds with no message (frame or heartbeat) received before the connection is reported STALE.")]
    public float connectionTimeoutSeconds = 6f;

    public event Action<List<LidarScanPoint>> OnScanReceived;
    /// <summary>Raised on the main thread with every accepted (see class remarks -- "Frame
    /// ordering") `PERCEPTION_FRAME`. The primary way TrackedObjectVisualizer/
    /// OccupancyMapRenderer/SafetyZoneRenderer/CollisionRiskIndicator/HUDController/
    /// RiskAudioController/VehiclePoseSync receive perception data.</summary>
    public event Action<PerceptionFrameData> OnPerceptionFrameReceived;
    public event Action<HeartbeatData> OnHeartbeatReceived;
    public event Action<SystemStatusData> OnSystemStatusReceived;
    public event Action<ErrorData> OnErrorReceived;
    /// <summary>Raised BEFORE any of the events above, exactly once per detected session boundary
    /// (see docs/architecture.md "Session and sequence management") -- every stateful visualizer
    /// (<see cref="Objects.TrackedObjectVisualizer"/> in particular: per-track_id views, "last
    /// seen" bookkeeping) must clear its own session-scoped state here, the same way
    /// `backend.state.LatestState.reset_for_new_session` does on the Python side, so a new
    /// scenario/hardware session never shows a stale leftover from the previous one.</summary>
    public event Action<string> OnSessionChanged;

    public ConnectionState State { get; private set; } = ConnectionState.Disconnected;

    /// <summary>Most recently *accepted* frame's `frame_id` -- exposed for a debug HUD; `-1`
    /// before the first frame.</summary>
    public long LastFrameId { get; private set; } = -1;
    public int DuplicateOrOutOfOrderFramesDropped { get; private set; }
    /// <summary>The session_id this client is currently tracking -- `null` until the first message
    /// that carries one arrives (see <see cref="SessionValidator"/>). Exposed for a debug HUD.</summary>
    public string CurrentSessionId { get; private set; }

    readonly Queue<MessageEnvelope> _incoming = new Queue<MessageEnvelope>();
    readonly object _lock = new object();

    Thread _thread;
    volatile bool _running;
    long? _lastAcceptedFrameId;
    string _supersededSessionId;
    float _lastMessageReceivedAt = -1f;

    void OnEnable()
    {
        _running = true;
        _thread = new Thread(ReadLoop) { IsBackground = true };
        _thread.Start();
    }

    void OnDisable()
    {
        _running = false;
        if (_thread != null && _thread.IsAlive)
            _thread.Join(500);
    }

    void ReadLoop()
    {
        while (_running)
        {
            TcpClient client = null;
            try
            {
                State = ConnectionState.Connecting;
                client = new TcpClient();
                client.Connect(host, port);
                State = ConnectionState.Connected;

                using (var stream = client.GetStream())
                using (var reader = new StreamReader(stream))
                {
                    while (_running && client.Connected)
                    {
                        string line = reader.ReadLine();
                        if (line == null) break; // remote closed the connection
                        if (string.IsNullOrWhiteSpace(line)) continue;

                        MessageEnvelope envelope = TryParseEnvelope(line);
                        if (envelope == null) continue; // malformed line -- skip, never crash (see docs/communication.md "Data validation")

                        lock (_lock)
                        {
                            _incoming.Enqueue(envelope);
                        }
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("PerceptionTCPClient: connection error: " + e.Message);
            }
            finally
            {
                if (client != null) client.Close();
            }

            if (!_running) break;

            State = ConnectionState.Reconnecting;
            Thread.Sleep(Mathf.Max(100, Mathf.RoundToInt(reconnectDelaySeconds * 1000f)));
        }

        State = ConnectionState.Disconnected;
    }

    static MessageEnvelope TryParseEnvelope(string line)
    {
        try
        {
            return JsonConvert.DeserializeObject<MessageEnvelope>(line);
        }
        catch (Exception e)
        {
            Debug.LogWarning("PerceptionTCPClient: failed to parse message envelope (" + e.GetType().Name + "): " + e.Message);
            return null;
        }
    }

    void Update()
    {
        List<MessageEnvelope> batch = null;
        lock (_lock)
        {
            if (_incoming.Count > 0)
            {
                batch = new List<MessageEnvelope>(_incoming);
                _incoming.Clear();
            }
        }

        if (batch != null)
        {
            foreach (var envelope in batch)
                Dispatch(envelope);
        }

        UpdateStaleness();
    }

    void Dispatch(MessageEnvelope envelope)
    {
        if (envelope == null || string.IsNullOrEmpty(envelope.messageType)) return;

        SessionStatus sessionStatus = SessionValidator.Classify(CurrentSessionId, _supersededSessionId, envelope.sessionId);
        if (!SessionValidator.IsAcceptable(sessionStatus))
        {
            // "Old sessions must never overwrite new sessions" -- a stale message somehow still
            // carrying an already-superseded session_id, rejected outright before touching any
            // state at all (see docs/architecture.md "Session and sequence management").
            Debug.LogWarning("PerceptionTCPClient: rejected message from superseded session_id=" + envelope.sessionId + " (current=" + CurrentSessionId + ").");
            return;
        }
        if (sessionStatus == SessionStatus.New)
        {
            Debug.Log("PerceptionTCPClient: new session detected: " + CurrentSessionId + " -> " + envelope.sessionId + " (source_id=" + envelope.sourceId + ") -- resetting session state.");
            _supersededSessionId = CurrentSessionId;
            CurrentSessionId = envelope.sessionId;
            _lastAcceptedFrameId = null; // this session's own frame_id sequence starts over -- see FrameIdValidator
            LastFrameId = -1;
            DuplicateOrOutOfOrderFramesDropped = 0;
            OnSessionChanged?.Invoke(envelope.sessionId); // subscribers must clear their own session-scoped state before this method returns
        }

        _lastMessageReceivedAt = Time.unscaledTime;
        if (State == ConnectionState.Stale) State = ConnectionState.Connected;

        switch (envelope.messageType)
        {
            case "PERCEPTION_FRAME":
                DispatchPerceptionFrame(envelope);
                break;
            case "HEARTBEAT":
                DispatchData<HeartbeatData>(envelope, OnHeartbeatReceived);
                break;
            case "SYSTEM_STATUS":
                DispatchData<SystemStatusData>(envelope, OnSystemStatusReceived);
                break;
            case "ERROR":
                DispatchData<ErrorData>(envelope, OnErrorReceived);
                break;
            default:
                // Unknown/future message type -- log once at debug level and ignore, never crash
                // (see docs/communication.md "Data validation": "unsupported protocol version" /
                // "unknown message type").
                Debug.Log("PerceptionTCPClient: ignoring unknown message_type '" + envelope.messageType + "'.");
                break;
        }
    }

    void DispatchPerceptionFrame(MessageEnvelope envelope)
    {
        if (!envelope.frameId.HasValue)
        {
            Debug.LogWarning("PerceptionTCPClient: PERCEPTION_FRAME message missing frame_id -- dropping.");
            return;
        }

        FrameIdStatus status = FrameIdValidator.Classify(_lastAcceptedFrameId, envelope.frameId.Value);
        if (!FrameIdValidator.IsAcceptable(status))
        {
            DuplicateOrOutOfOrderFramesDropped++;
            return; // never let an older/duplicate frame overwrite already-applied state
        }

        PerceptionFrameData frame;
        try
        {
            frame = envelope.data != null ? envelope.data.ToObject<PerceptionFrameData>() : null;
        }
        catch (Exception e)
        {
            Debug.LogWarning("PerceptionTCPClient: failed to parse PERCEPTION_FRAME payload: " + e.Message);
            return;
        }
        if (frame == null) return;

        _lastAcceptedFrameId = envelope.frameId.Value;
        LastFrameId = envelope.frameId.Value;

        OnPerceptionFrameReceived?.Invoke(frame);

        if (frame.points != null && frame.points.Count > 0)
        {
            var scanPoints = new List<LidarScanPoint>(frame.points.Count);
            foreach (var p in frame.points)
            {
                if (float.IsNaN(p.distance) || float.IsInfinity(p.distance)) continue; // never let bad data reach the visualizer
                if (float.IsNaN(p.angle) || float.IsInfinity(p.angle)) continue;
                scanPoints.Add(new LidarScanPoint(p.angle, p.distance, p.valid));
            }
            OnScanReceived?.Invoke(scanPoints);
        }
    }

    void DispatchData<T>(MessageEnvelope envelope, Action<T> handler) where T : class
    {
        if (handler == null || envelope.data == null) return;
        try
        {
            T data = envelope.data.ToObject<T>();
            if (data != null) handler.Invoke(data);
        }
        catch (Exception e)
        {
            Debug.LogWarning("PerceptionTCPClient: failed to parse " + envelope.messageType + " payload: " + e.Message);
        }
    }

    void UpdateStaleness()
    {
        if (State != ConnectionState.Connected && State != ConnectionState.Stale) return;
        if (_lastMessageReceivedAt < 0f) return; // nothing received yet since (re)connecting -- not stale, just new

        bool timedOut = (Time.unscaledTime - _lastMessageReceivedAt) > connectionTimeoutSeconds;
        State = timedOut ? ConnectionState.Stale : ConnectionState.Connected;
    }

    void OnApplicationQuit()
    {
        _running = false;
    }
}
