using UnityEngine;

/// <summary>Draws Python-perception-frame coordinate axes (forward/left, per
/// CoordinateConverter) at <see cref="origin"/>, for the debug-mode "Coordinate axes" toggle.
/// Uses <c>Debug.DrawRay</c> so it costs nothing when disabled and never allocates a
/// GameObject.</summary>
public class CoordinateAxesGizmo : MonoBehaviour
{
    public Transform origin;
    public float axisLength = 2f;
    public bool show = true;

    void Update()
    {
        if (!show || origin == null) return;

        Vector3 forward = CoordinateConverter.PerceptionToUnity(axisLength, 0f, origin, 0f) - origin.position;
        Vector3 left = CoordinateConverter.PerceptionToUnity(0f, axisLength, origin, 0f) - origin.position;

        Debug.DrawRay(origin.position, forward, Color.blue);                       // Python +X (forward)
        Debug.DrawRay(origin.position, left, Color.green);                          // Python +Y (left)
        Debug.DrawRay(origin.position, Vector3.up * axisLength * 0.5f, Color.red);  // Unity up (no Python equivalent -- 2D sensor)
    }
}
