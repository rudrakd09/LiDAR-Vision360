using UnityEngine;

/// <summary>
/// Switches between a top-down camera (essential for verifying the 360° LiDAR, occupancy map,
/// safety zones, and trajectories -- see docs/unity.md "Camera") and a third-person/isometric
/// follow camera. Assign two pre-positioned <see cref="Camera"/> GameObjects in the Editor; this
/// script only enables one at a time and does not move/animate either camera itself, keeping
/// framing fully Editor-configurable.
/// </summary>
public class CameraModeController : MonoBehaviour
{
    public enum CameraMode
    {
        TopDown,
        ThirdPerson,
    }

    [Header("Cameras")]
    public Camera topDownCamera;
    public Camera thirdPersonCamera;

    [Header("Mode")]
    public CameraMode mode = CameraMode.TopDown;
    [Tooltip("Keyboard key to cycle camera modes during Play mode.")]
    public KeyCode cycleKey = KeyCode.Tab;

    void Start()
    {
        Apply();
    }

    void Update()
    {
        if (Input.GetKeyDown(cycleKey))
            Cycle();
    }

    public void Cycle()
    {
        mode = mode == CameraMode.TopDown ? CameraMode.ThirdPerson : CameraMode.TopDown;
        Apply();
    }

    public void SetMode(CameraMode newMode)
    {
        mode = newMode;
        Apply();
    }

    void Apply()
    {
        if (topDownCamera != null) topDownCamera.gameObject.SetActive(mode == CameraMode.TopDown);
        if (thirdPersonCamera != null) thirdPersonCamera.gameObject.SetActive(mode == CameraMode.ThirdPerson);
    }
}
