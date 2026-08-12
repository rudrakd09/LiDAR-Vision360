using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Debug-mode visibility toggles for every optional visualization layer -- raw LiDAR points,
/// classified objects, track IDs (labels), velocity vectors, predicted trajectories, occupancy
/// map, safety zones, clearance, coordinate axes. See docs/unity.md "Debug mode".
///
/// Each <see cref="Toggle"/> is optional (assign only the ones you actually add to your debug
/// panel) and flips the relevant layer's own <c>GameObject.SetActive</c> -- not the component's
/// <c>enabled</c> flag alone, which would stop it from updating but leave whatever it already
/// rendered (pooled points, LineRenderers, the map texture) visibly stuck on screen. This script
/// owns no visualization logic itself.
/// </summary>
public class DebugTogglesPanel : MonoBehaviour
{
    [Header("Targets")]
    public LidarPointCloudRenderer pointCloudRenderer;
    public TrackedObjectVisualizer trackedObjectVisualizer;
    public OccupancyMapRenderer occupancyMapRenderer;
    public SafetyZoneRenderer safetyZoneRenderer;
    public CoordinateAxesGizmo coordinateAxesGizmo;
    public GameObject clearanceUIRoot;

    [Header("UI Toggles (optional -- wire via Inspector)")]
    public Toggle rawPointsToggle;
    public Toggle classifiedObjectsToggle;
    public Toggle trackIdsToggle;
    public Toggle velocityVectorsToggle;
    public Toggle predictedTrajectoriesToggle;
    public Toggle occupancyMapToggle;
    public Toggle safetyZonesToggle;
    public Toggle clearanceToggle;
    public Toggle coordinateAxesToggle;

    void Start()
    {
        WireToggle(rawPointsToggle, SetPointCloudVisible);
        WireToggle(classifiedObjectsToggle, SetObjectsVisible);
        WireToggle(trackIdsToggle, SetLabelsVisible);
        WireToggle(velocityVectorsToggle, SetVelocityVectorsVisible);
        WireToggle(predictedTrajectoriesToggle, SetTrajectoriesVisible);
        WireToggle(occupancyMapToggle, SetOccupancyMapVisible);
        WireToggle(safetyZonesToggle, SetSafetyZonesVisible);
        WireToggle(clearanceToggle, v => { if (clearanceUIRoot != null) clearanceUIRoot.SetActive(v); });
        WireToggle(coordinateAxesToggle, v => { if (coordinateAxesGizmo != null) coordinateAxesGizmo.show = v; });
    }

    void WireToggle(Toggle toggle, System.Action<bool> apply)
    {
        if (toggle == null) return;
        apply(toggle.isOn);
        toggle.onValueChanged.AddListener(new UnityEngine.Events.UnityAction<bool>(apply));
    }

    void SetPointCloudVisible(bool visible)
    {
        if (pointCloudRenderer != null) pointCloudRenderer.gameObject.SetActive(visible);
    }

    void SetObjectsVisible(bool visible)
    {
        if (trackedObjectVisualizer != null) trackedObjectVisualizer.gameObject.SetActive(visible);
    }

    void SetOccupancyMapVisible(bool visible)
    {
        if (occupancyMapRenderer != null) occupancyMapRenderer.gameObject.SetActive(visible);
    }

    void SetSafetyZonesVisible(bool visible)
    {
        if (safetyZoneRenderer != null) safetyZoneRenderer.gameObject.SetActive(visible);
    }

    void SetLabelsVisible(bool visible)
    {
        if (trackedObjectVisualizer == null) return;
        foreach (var view in trackedObjectVisualizer.GetComponentsInChildren<TrackedObjectView>(true))
            view.SetLabelVisible(visible);
    }

    void SetVelocityVectorsVisible(bool visible)
    {
        if (trackedObjectVisualizer == null) return;
        foreach (var view in trackedObjectVisualizer.GetComponentsInChildren<TrackedObjectView>(true))
            view.showVelocityVector = visible; // re-checked every ApplyData() call -- takes effect on the next scan
    }

    void SetTrajectoriesVisible(bool visible)
    {
        if (trackedObjectVisualizer == null) return;
        foreach (var trajectory in trackedObjectVisualizer.GetComponentsInChildren<TrajectoryRenderer>(true))
            trajectory.trajectoryEnabled = visible;
    }
}
