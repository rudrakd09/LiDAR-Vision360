using UnityEngine;

/// <summary>
/// Applies Python's own vehicle pose (<c>frame.vehicle</c>) to this transform every scan --
/// position and heading only, never computed/estimated in Unity (see docs/unity.md "Coordinate
/// transformation"). Every other script in this project that needs a vehicle reference
/// (LidarPointCloudRenderer, TrackedObjectVisualizer, OccupancyMapRenderer, SafetyZoneRenderer,
/// CoordinateAxesGizmo -- all via their own <c>origin</c> field) should point at the <b>same</b>
/// Transform this component drives, so moving the vehicle here automatically moves every other
/// visualization consistently.
///
/// <para>Currently a no-op in practice: nothing on the Python side (Phase 9's own scope) ever
/// supplies a non-identity <c>VehicleState</c> to <c>scripts/serve_unity_bridge.py</c>, so
/// <c>frame.vehicle</c> is always <c>(0, 0, 0)</c> today. Included now so the architecture is
/// already correct once a future phase (vehicle localization / real hardware odometry) starts
/// supplying real values -- no Unity-side changes will be needed then.</para>
/// </summary>
public class VehiclePoseSync : MonoBehaviour
{
    public PerceptionTCPClient client;
    [Tooltip("If false, only heading/rotation is applied -- useful when this transform's position is meant to stay fixed at the scene's authored origin regardless of Python's reported x/y.")]
    public bool applyPosition = true;
    public bool applyHeading = true;

    void OnEnable()
    {
        if (client != null) client.OnPerceptionFrameReceived += HandleFrame;
    }

    void OnDisable()
    {
        if (client != null) client.OnPerceptionFrameReceived -= HandleFrame;
    }

    void HandleFrame(PerceptionFrameData frame)
    {
        if (frame == null || frame.vehicle == null) return;

        if (applyPosition)
            transform.position = CoordinateConverter.PerceptionToUnity(frame.vehicle.x, frame.vehicle.y, null, transform.position.y);

        if (applyHeading)
            transform.rotation = Quaternion.Euler(0f, CoordinateConverter.PerceptionHeadingToUnityYaw(frame.vehicle.heading), 0f);
    }
}
