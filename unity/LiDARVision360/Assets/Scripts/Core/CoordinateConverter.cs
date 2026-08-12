using UnityEngine;

/// <summary>
/// The ONE place coordinate conversion between Python perception space and Unity world space
/// happens. Every new script in this project must go through this class rather than re-deriving
/// its own cos/sin math -- see docs/unity.md "Coordinate transformation" for the full derivation,
/// a worked numeric example, and why this specific mapping (not some other, seemingly more
/// "natural" one) is correct for this project.
///
/// <para><b>Python perception convention</b> (docs/coordinates.md): <c>+X</c> forward,
/// <c>+Y</c> left, <c>angle</c> in degrees CCW from <c>+X</c>, meters, vehicle/LiDAR at the
/// world origin under the identity pose. <c>x = distance * cos(angle)</c>,
/// <c>y = distance * sin(angle)</c> (the exact formula
/// <c>perception/src/coordinates/transformer.py</c> already implements).</para>
///
/// <para><b>This project's existing Unity convention, as established by the already-working
/// LidarCubes.cs (unmodified, see Visualization/LidarCubes.cs)</b>:
/// <c>x = distance*cos(angleRad); z = distance*sin(angleRad);</c> placed directly on Unity's own
/// X/Z axes. Reading that formula against the Python formula above line for line shows it is
/// already a direct, <i>unrotated</i> relabeling -- Unity's X axis is standing in for Python's
/// forward axis, and Unity's Z axis for Python's left axis:</para>
///
/// <code>
///   Python X (forward)  -&gt;  Unity X
///   Python Y (left)     -&gt;  Unity Z
///   Python has no Z      -&gt;  Unity Y  (fixed height only -- 2D LiDAR, no elevation data,
///                                       see docs/architecture.md "Sensor limitation")
/// </code>
///
/// <para>This was verified by deriving it directly from LidarCubes.cs's own working code, not
/// assumed from a "standard" Unity convention (where +Z is usually "forward") -- per this
/// phase's explicit instruction not to blindly change the existing formula. One practical
/// consequence: any vehicle/camera/prefab built for this scene should be authored/oriented so its
/// own "forward" visually points along world <b>+X</b>, not Unity's native <c>transform.forward</c>
/// (+Z) -- see <see cref="PerceptionHeadingToUnityYaw"/> for the heading/rotation implication of
/// this, including where that formula's own assumption may need a per-prefab adjustment.</para>
/// </summary>
public static class CoordinateConverter
{
    /// <summary>Fixed height (Unity Y) new perception-driven visuals are placed at by default --
    /// matches LidarCubes.cs's own hard-coded <c>0.25f</c> point height.</summary>
    public const float DefaultHeight = 0.25f;

    /// <summary>Python (x forward, y left) meters -&gt; Unity world position, relative to
    /// <paramref name="origin"/> (typically the vehicle/LiDAR-mount transform; pass
    /// <c>null</c> to get a world-space point with no additional origin offset/rotation
    /// applied).</summary>
    public static Vector3 PerceptionToUnity(float pythonX, float pythonY, Transform origin, float height = DefaultHeight)
    {
        Vector3 local = new Vector3(pythonX, height, pythonY);
        return origin != null ? origin.TransformPoint(local) : local;
    }

    /// <summary>Python polar (angle degrees CCW from +X, distance meters) -&gt; Unity world
    /// position. Equivalent to
    /// <c>PerceptionToUnity(distance*cos(angleRad), distance*sin(angleRad), ...)</c> -- provided
    /// directly since this is the exact form raw LiDAR points (angle, distance) arrive in, and
    /// is what LidarPointCloudRenderer.cs uses instead of re-deriving the trig itself.</summary>
    public static Vector3 PolarToUnity(float angleDeg, float distanceM, Transform origin, float height = DefaultHeight)
    {
        float angleRad = angleDeg * Mathf.Deg2Rad;
        float pythonX = distanceM * Mathf.Cos(angleRad);
        float pythonY = distanceM * Mathf.Sin(angleRad);
        return PerceptionToUnity(pythonX, pythonY, origin, height);
    }

    /// <summary>Python 2D velocity (vx forward, vy left, m/s) -&gt; Unity world-space direction
    /// vector (rotation only, no translation -- this is a vector, not a position).</summary>
    public static Vector3 PerceptionVelocityToUnity(float vx, float vy, Transform origin)
    {
        Vector3 local = new Vector3(vx, 0f, vy);
        return origin != null ? origin.TransformDirection(local) : local;
    }

    /// <summary>
    /// Python heading (degrees, CCW from +X, same convention as <c>LiDARPoint.angle</c> and
    /// <c>VehiclePose.heading</c>) -&gt; a Unity Y-axis Euler yaw, <b>assuming the target
    /// GameObject's own unrotated mesh already visually points along local +X</b> (matching the
    /// position-mapping convention above, so a heading of <c>0</c> needs no additional
    /// rotation).
    ///
    /// <para><b>This assumption cannot be verified without a live Unity Editor / Scene view in
    /// this environment</b> -- if your prefab's authored "front" points along a different local
    /// axis (many default primitives/arrow assets point along +Z or +Y instead), the resulting
    /// rotation will be off by a fixed amount. Every script that calls this (TrackedObjectView,
    /// a future vehicle controller) exposes its own <c>yawOffsetDeg</c> Inspector field for
    /// exactly this correction -- rotate one object in Play mode, read off the visual error in
    /// degrees, and set that field once. See docs/unity.md "Coordinate transformation" /
    /// "Known limitations".</para>
    /// </summary>
    public static float PerceptionHeadingToUnityYaw(float headingDeg)
    {
        return -headingDeg;
    }
}
