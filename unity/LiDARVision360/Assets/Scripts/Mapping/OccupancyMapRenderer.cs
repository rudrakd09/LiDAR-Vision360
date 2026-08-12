using UnityEngine;

/// <summary>
/// Renders the Phase 8 occupancy map as a single textured quad -- <b>not</b> one GameObject per
/// cell (which would mean up to 160,000 GameObjects at full resolution; this phase explicitly
/// forbids that, see "Map performance"). Decodes the base64-packed cell-state bytes Python sends
/// (<c>perception/src/serialization/unity_protocol.pack_occupancy_grid</c>) directly into a
/// <see cref="Texture2D"/> via <c>SetPixels32</c>/<c>Apply</c>, reusing the same texture instance
/// across updates (never allocating a new one per frame) unless the map's own dimensions change.
/// See docs/unity.md "Occupancy map visualization".
///
/// <para><b>Texture orientation is a best-effort, not visually verified</b> (no live Unity Editor
/// in the environment this was built in) -- <see cref="flipRows"/>/<see cref="flipColumns"/> are
/// provided as a one-click fix: run the scene with the point cloud and this map both visible, and
/// if the occupied cells appear mirrored relative to where the actual obstacles/points are,
/// toggle the relevant flag. See docs/unity.md "Known limitations".</para>
/// </summary>
[RequireComponent(typeof(MeshRenderer))]
public class OccupancyMapRenderer : MonoBehaviour
{
    [Header("Source")]
    public PerceptionTCPClient client;
    public Transform origin;

    [Header("Colors (models.mapping.CellState: 0=UNKNOWN, 1=FREE, 2=OCCUPIED)")]
    public Color unknownColor = new Color(0.5f, 0.5f, 0.5f, 0.35f);
    public Color freeColor = new Color(1f, 1f, 1f, 0.15f);
    public Color occupiedColor = new Color(0f, 0f, 0f, 0.85f);

    [Header("Placement")]
    [Tooltip("Unity Y height the map plane is rendered at -- kept low so it reads as a ground overlay, not floating above obstacles.")]
    public float planeHeight = 0.02f;

    [Header("Orientation (see class remarks -- adjust if the rendered map looks mirrored)")]
    public bool flipRows = false;
    public bool flipColumns = false;

    Texture2D _texture;
    int _textureWidth = -1;
    int _textureHeight = -1;
    MeshRenderer _renderer;
    Color32[] _pixelBuffer;

    void Awake()
    {
        _renderer = GetComponent<MeshRenderer>();
    }

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
        // Most scans don't include a map update (see scripts/serve_unity_bridge.py
        // --map-every-n-scans) -- simply keep showing the last one rendered, don't clear.
        if (frame?.map == null) return;

        OccupancyMapData map = frame.map;
        if (string.IsNullOrEmpty(map.cellsBase64) || map.widthCells <= 0 || map.heightCells <= 0) return;

        byte[] cells;
        try
        {
            cells = System.Convert.FromBase64String(map.cellsBase64);
        }
        catch (System.FormatException e)
        {
            Debug.LogWarning("OccupancyMapRenderer: malformed base64 map payload: " + e.Message);
            return;
        }

        int expected = map.widthCells * map.heightCells;
        if (cells.Length != expected)
        {
            Debug.LogWarning($"OccupancyMapRenderer: map payload size mismatch (got {cells.Length} bytes, expected {expected}) -- skipping this update.");
            return;
        }

        EnsureTexture(map.widthCells, map.heightCells);
        FillPixelBuffer(cells, map.widthCells, map.heightCells);
        _texture.SetPixels32(_pixelBuffer);
        _texture.Apply(false);

        PositionPlane(map);
    }

    void FillPixelBuffer(byte[] cells, int width, int height)
    {
        for (int row = 0; row < height; row++)
        {
            int srcRow = flipRows ? (height - 1 - row) : row;
            int rowBase = row * width;
            int srcRowBase = srcRow * width;
            for (int col = 0; col < width; col++)
            {
                int srcCol = flipColumns ? (width - 1 - col) : col;
                _pixelBuffer[rowBase + col] = ColorForCellState(cells[srcRowBase + srcCol]);
            }
        }
    }

    void EnsureTexture(int width, int height)
    {
        if (_texture != null && _textureWidth == width && _textureHeight == height) return;

        _texture = new Texture2D(width, height, TextureFormat.RGBA32, false);
        _texture.filterMode = FilterMode.Point; // crisp cell boundaries, not blurred
        _texture.wrapMode = TextureWrapMode.Clamp;
        _textureWidth = width;
        _textureHeight = height;
        _pixelBuffer = new Color32[width * height];

        if (_renderer.sharedMaterial == null)
            _renderer.material = new Material(Shader.Find("Unlit/Transparent"));
        _renderer.material.mainTexture = _texture;
    }

    void PositionPlane(OccupancyMapData map)
    {
        float widthM = map.widthCells * map.resolutionM;
        float heightM = map.heightCells * map.resolutionM;
        float centerPythonX = map.originXM + widthM / 2f;
        float centerPythonY = map.originYM + heightM / 2f;

        transform.position = CoordinateConverter.PerceptionToUnity(centerPythonX, centerPythonY, origin, planeHeight);
        // A default Unity Quad is 1x1 in its own local XY plane; laid flat (rotated 90 deg on X)
        // its local X/Y axes become world X/Z -- matching CoordinateConverter's own X=forward,
        // Z=left convention when scaled by (widthM, heightM) here. See docs/unity.md "Occupancy
        // map visualization" for the exact quad-orientation this assumes (a Quad primitive,
        // rotated -- see the Prefabs setup instructions).
        transform.localScale = new Vector3(widthM, heightM, 1f);
    }

    Color32 ColorForCellState(byte state)
    {
        switch (state)
        {
            case 1: return freeColor;
            case 2: return occupiedColor;
            default: return unknownColor;
        }
    }
}
