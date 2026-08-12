using System;
using System.Collections.Generic;

/// <summary>One raw LiDAR measurement -- angle in degrees, distance in meters, matching this
/// project's Python-side <c>models.lidar.LiDARPoint</c> convention exactly (see
/// docs/coordinates.md). No unit conversion is needed once a point reaches this struct; each
/// concrete data source is responsible for converting its own wire units into this one on the
/// way in.</summary>
public struct LidarScanPoint
{
    public float angleDeg;
    public float distanceM;
    public bool valid;

    public LidarScanPoint(float angleDeg, float distanceM, bool valid = true)
    {
        this.angleDeg = angleDeg;
        this.distanceM = distanceM;
        this.valid = valid;
    }
}

/// <summary>Shared by every data source and the HUD's connection-status display -- see
/// docs/communication.md "Heartbeat / connection status". <see cref="Stale"/> (added Phase 12) is
/// distinct from a clean TCP disconnect: the socket is still open, but no message (frame or
/// heartbeat) has arrived within `connectionTimeoutSeconds` -- something between the two
/// processes has stopped working without either side's OS-level connection actually
/// dropping.</summary>
public enum ConnectionState
{
    Disconnected,
    Connecting,
    Connected,
    Stale,
    Reconnecting,
}

/// <summary>
/// Common abstraction over anything that can produce LiDAR scans for the visualization layer --
/// today, the structured JSON perception stream (<see cref="PerceptionTCPClient"/>); in the
/// future, a WebSocket or cloud-replay source, per this phase's own "allows future Python/
/// WebSocket/cloud data sources" goal (see docs/unity.md "Input abstraction").
///
/// <para>The existing, working <c>LidarSerialReader</c>/<c>LidarTCPClient</c> scripts are
/// deliberately <b>not</b> modified to implement this interface. Both hard-reference the
/// concrete <c>LidarCubes</c>/<c>LidarBeep</c> classes directly in their own public fields
/// (<c>public LidarCubes visualizer;</c>) rather than any abstraction, and retrofitting them
/// would mean either changing those field types (risking every existing Inspector wiring anyone
/// already has) or subclassing them (unnecessary complexity for zero behavioral gain, since
/// nothing about their own logic needs to change). Instead, <see cref="LidarInputManager"/>
/// selects between the untouched legacy pipeline (Serial/raw-TCP straight into
/// LidarCubes/LidarBeep, unchanged) and this new pipeline (PerceptionTCPClient into the new
/// visualization suite) at the top level -- see docs/unity.md "Live scan update" for the full
/// reasoning.</para>
/// </summary>
public interface ILidarDataSource
{
    /// <summary>Raised whenever a new full scan is available (only when the underlying frame
    /// actually included raw points -- see docs/unity.md "Python -> Unity protocol", the
    /// <c>points</c> field is optional and off by default for bandwidth).</summary>
    event Action<List<LidarScanPoint>> OnScanReceived;

    ConnectionState State { get; }
}
