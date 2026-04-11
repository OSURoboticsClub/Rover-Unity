using Newtonsoft.Json;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Networking;

public interface ITileLoader
{
    void LoadTile(int x, int y, int zoom, Action<Texture2D> onLoaded);
}

// Loads from Assets folder (Editor Only) or Resources (Build)
public class LocalTileLoader : ITileLoader
{
    private string _baseFolder;

    public LocalTileLoader(string folder)
    {
        _baseFolder = folder;
    }

    public void LoadTile(int x, int y, int zoom, Action<Texture2D> onLoaded)
    {
#if UNITY_EDITOR
        string path = $"Assets/{_baseFolder}/~{x},{y}~.png";
        Debug.Log($"Loading tile: {path}");
        Texture2D tex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>(path);

        if (tex != null)
        {
            onLoaded?.Invoke(tex);
        }
        else
        {
            Debug.LogWarning($"[LocalLoader] Tile not found: {path}");
        }
#else
        Debug.LogError("AssetDatabase loading is Editor-only. Use Resources.Load for builds.");
#endif
    }
}

// Load from a Web API (e.g., OpenStreetMap, Mapbox, or custom server)
public class WebTileLoader : ITileLoader
{
    private string _urlTemplate; // e.g. "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

    public WebTileLoader(string urlTemplate)
    {
        _urlTemplate = urlTemplate;
    }

    public void LoadTile(int x, int y, int zoom, Action<Texture2D> onLoaded)
    {
        // Web requests must be run via a Coroutine. 
        // We need a MonoBehaviour to run coroutines, so we find a host.
        SatelliteMapSystem.Instance.StartCoroutine(DownloadTile(x, y, zoom, onLoaded));
    }

    private IEnumerator DownloadTile(int x, int y, int zoom, Action<Texture2D> onLoaded)
    {
        // Calculate the Global Web Mercator index if needed. 
        // NOTE: local -18 to 17 indices are RELATIVE. 
        // If using a real API, you must convert relative X/Y to Global X/Y here.
        // For this example, we assume the URL expects the relative index or you have a custom server.

        string url = _urlTemplate
            .Replace("{x}", x.ToString())
            .Replace("{y}", y.ToString())
            .Replace("{z}", zoom.ToString());

        using (UnityWebRequest uwr = UnityWebRequestTexture.GetTexture(url))
        {
            yield return uwr.SendWebRequest();

            if (uwr.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError($"[WebLoader] Error: {uwr.error} ({url})");
            }
            else
            {
                onLoaded?.Invoke(DownloadHandlerTexture.GetContent(uwr));
            }
        }
    }
}


// ------------------------------
// MISSION CONFIGURATION CLASSES


[Serializable]
public struct Waypoint
{
    public double latitude;
    public double longitude;
    public Waypoint(double lat, double lon)
    {
        latitude = lat;
        longitude = lon;
    }
}

[Serializable]
public class MissionConfig
{
    // Match search patterns and objects to ROS2 enums.
    public enum SearchPattern { None, Spiral, Lawnmower }
    public enum SearchObject { None, Mallet, Hammer, Waterbottle, Aruco }

    public List<Waypoint> waypoints = new List<Waypoint>();
    public SearchPattern searchPattern;
    public SearchObject searchObject;
    public float searchParam1 = 2.0f; // Spiral/Lawnmower: Lane Spacing (Meters)
    public float searchParam2 = 1.0f; // Spiral: Max Radius | Lawnmower: Step Size (Meters)

    public MissionConfig(SearchPattern pattern, SearchObject obj, float param1, float params2, List<Waypoint> points)
    {
        searchPattern = pattern;
        searchObject = obj;
        searchParam1 = param1;
        searchParam2 = params2;
        waypoints = points;
    }

    public string ToJson()
    {
        var data = new
        {
            search_object = (int)searchObject,
            search_pattern = (int)searchPattern,
            search_param_1 = searchParam1,
            search_param_2 = searchParam2,
            nav_waypoints = waypoints.ConvertAll(wp => new { latitude = wp.latitude, longitude = wp.longitude })
        };
        return JsonConvert.SerializeObject(data, Formatting.Indented);
    }
}

public enum MissionState
{
    IDLE = 0,
    MOVING_TO_START = 1,
    SEARCHING = 2,
    WAITING_FOR_NAV_IDLE = 3,
    INVESTIGATING = 4,
    RETURNING_TO_SEARCH = 5,
    SUCCESS = 6,
    FAILED = 7,
    RETURNING_HOME = 8,
    STOPPED = 9
}

[System.Serializable]
public class MissionTelemetry
{
    public double timestamp;
    public MissionState mission_state;
    public double latitude;
    public double longitude;
    public float heading;
}

// ---------------------------------
// MISSION VISUALIZATION COMPONENTS

public class WaypointHoverEffect : MonoBehaviour
{
    private int _waypointIndex;
    private SpriteRenderer _sr;
    private Material _mat;
    private float _scaleFactor = 1.2f;

    public void SetBaseColor(Color color, int index)
    {
        _sr = GetComponent<SpriteRenderer>();
        _sr.color = color;
        _mat = _sr.material;
        _mat.EnableKeyword("_EMISSION");
        _waypointIndex = index;
    }
    void OnMouseEnter()
    {
        if (SatelliteMapSystem.Instance.IsDragging() && SatelliteMapSystem.Instance.GetDraggedIndex() != _waypointIndex)
            return;
        _mat.SetColor("_EmissionColor", _sr.color * 1.5f);
        transform.localScale *= _scaleFactor;
        SatelliteMapSystem.Instance.TriggerHoverStart(_waypointIndex);
    }

    void OnMouseExit()
    {
        if (SatelliteMapSystem.Instance.IsDragging() && SatelliteMapSystem.Instance.GetDraggedIndex() == _waypointIndex)
            return;
        RemoveEffect();
    }

    public void RemoveEffect()
    {
        _mat.SetColor("_EmissionColor", Color.black);
        transform.localScale /= _scaleFactor;
        SatelliteMapSystem.Instance.TriggerHoverEnd(_waypointIndex);
    }
}

// -------------------------
// MAIN MAP CONTROLLER

[RequireComponent(typeof(LineRenderer))]
public class SatelliteMapSystem : MonoBehaviour
{
    public static SatelliteMapSystem Instance;

    public enum LoadMode { Local, Web }

    [ContextMenu("Export Mission to Console")]
    public void PrintMissionToConsole()
    {
        if (currentMission == null) return;
        string json = currentMission.ToJson();
        Debug.Log($"<color=lime><b>[MISSION EXPORT]</b></color>\n{json}");
    }

    public string GetMissionJson()
    {
        if (currentMission == null) return null;
        string json = currentMission.ToJson();
        Debug.Log($"<color=lime><b>[MISSION EXPORT]</b></color>\n{json}");
        return json;
    }

    [Header("Configuration")]
    public LoadMode mode = LoadMode.Local;
    public string localTileFolder = "Control Interface App/Maps/URC";
    [Tooltip("Use {x}, {y}, {z} as placeholders")]
    public string webUrlTemplate = "https://server.com/tiles/{z}/{x}/{y}.png";
    public GameObject mapOrigin;
    public RectTransform mapUIPanel;

    [Header("Grid Settings")]
    public int minX = -18;
    public int maxX = 17;
    public int minY = -18;
    public int maxY = 17;

    [Header("Geospatial Settings")]
    public int zoomLevel = 19;
    public double originLat = 38.406435;
    public double originLon = -110.792015;

    [Header("Visual Settings")]
    public float tileSize = 256.0f; // Size of the tile in Unity Units
    public Material baseMaterial;
    public Sprite pathMarker;
    public Sprite startMarker;
    public Sprite goalMarker;
    public Sprite boundsMarker;

    public Color startColor = Color.green;
    public Color waypointColor = Color.white;
    public Color goalColor = Color.red;

    public GameObject markerPrefab;
    public GameObject roverMarkerPrefab;

    [Range(0.1f, 5.0f), SerializeField]
    private float markerScale = 1.0f;
    public float MarkerScale
    {
        get => markerScale;
        set
        {
            if (markerScale != value)
            {
                markerScale = value;
                UpdateVisualScale();
            }
        }
    }
    private float _previousMarkerScale = 1.0f;

    private GameObject _markers;
    private GameObject _roverMarker;
    private LineRenderer _pathRenderer;
    private List<GameObject> _waypointMarkers = new List<GameObject>();
    private int _lastWaypointCount = 0;
    private GameObject _draggedMarker = null;
    private int _draggedIndex = -1;

    [Header("Camera Control")]
    [SerializeField] private Camera _cam;
    private CameraClearFlags _mainCamClearFlags;
    public float panSpeed = 1.0f;
    public float zoomSensitivity = 10.0f;
    public float minCamHeight = 5.0f;
    public float maxCamHeight = 500.0f;
    private float _lastCamHeight;


    // Internal State
    private ITileLoader _loader;
    private Vector3 _lastMousePos;
    private double _originGlobalTileX;
    private double _originGlobalTileY;
    private Vector3 _dragOffset;

    [Header("Mission Configuration (Override)")]
    [SerializeField] private MissionConfig.SearchPattern inspectorSearchPattern = MissionConfig.SearchPattern.None;
    [SerializeField] private MissionConfig.SearchObject buttonSearchObject;
    [Tooltip("Lawnmower/Spiral: Lane Spacing (Meters)")]
    [SerializeField] private float inspectorSearchParam1 = 2.0f;
    [Tooltip("Lawnmower: Step Size | Spiral: Max Radius (Meters)")]
    [SerializeField] private float inspectorSearchParam2 = 1.0f;
    private MissionConfig currentMission;


    // UI Action / Hooks
    public Action OnWaypointsChanged;
    public Action<int> OnWaypointHoverStart;
    public Action<int> OnWaypointHoverEnd;
    public Action<int, Waypoint> OnWaypointDragged;
    public Action<MissionTelemetry> OnTelemetryReceived;

    public void TriggerHoverStart(int index) => OnWaypointHoverStart?.Invoke(index);
    public void TriggerHoverEnd(int index) => OnWaypointHoverEnd?.Invoke(index);

    public bool IsDragging() => _draggedMarker != null;
    public int GetDraggedIndex() => _draggedIndex;

    private void Awake()
    {
        Instance = this;
        Camera[] allCams = Resources.FindObjectsOfTypeAll<Camera>();
        foreach (Camera cam in allCams)
        {
            if (cam.name == "Satellite Camera" && cam.gameObject.scene.isLoaded)
            {

                Debug.Log("Found cam " + cam.name);
                _cam = cam;
                break;
            }
        }
        if (_cam == null)
        {
            Debug.LogError("Satellite Camera not found.");
        }
        _mainCamClearFlags = Camera.main.clearFlags;
        _lastCamHeight = _cam.transform.position.y;

        _pathRenderer = GetComponent<LineRenderer>();
        _pathRenderer.startWidth = markerScale * 0.5f;
        _pathRenderer.endWidth = markerScale * 0.5f;
        _pathRenderer.alignment = LineAlignment.View;
        _pathRenderer.numCornerVertices = 15;
        _pathRenderer.numCapVertices = 10;
        _pathRenderer.positionCount = 0;
        _pathRenderer.useWorldSpace = true;
        _pathRenderer.sortingOrder = -2;
        baseMaterial = new Material(Shader.Find("Unlit/Texture"));

        // Initialize Loader
        if (mode == LoadMode.Local)
            _loader = new LocalTileLoader(localTileFolder);
        else
            _loader = new WebTileLoader(webUrlTemplate);

        // Pre-calculate Global Web Mercator Coordinates for the Origin (Lat/Lon)
        // This makes the click-to-GPS math fast later.
        _originGlobalTileX = LongitudeToTileX(originLon, zoomLevel);
        _originGlobalTileY = LatitudeToTileY(originLat, zoomLevel);

        currentMission = new MissionConfig(
            inspectorSearchPattern,
            buttonSearchObject,
            inspectorSearchParam1,
            inspectorSearchParam2,
            new List<Waypoint>()
        );
    }

    private bool _initalized = false;
    public void InitializeMap()
    {
        if (_initalized)
            return;
        GenerateMap();
        _initalized = true;
    }

    private void Start()
    {
        GenerateMap();

        _markers = new GameObject("Markers");
        _markers.transform.parent = this.transform;
        _markers.transform.localPosition = new Vector3(0, 0, 0);
        _roverMarker = SpawnMarkerAtGPS(originLat, originLon, "RoverMarker", _markers.transform, roverMarkerPrefab);

        // Center Camera
        if (_cam != null)
        {
            _cam.transform.position = new Vector3(0, 50, 0);
            _cam.transform.LookAt(Vector3.zero);
        }
    }

    private void Update()
    {
        HandleCameraInput();
        HandleInteraction();

        if (currentMission.waypoints.Count != _lastWaypointCount)
        {
            RenderWaypoints();
        }
    }

    private void LateUpdate()
    {
        if (!Mathf.Approximately(_cam.transform.position.y, _lastCamHeight))
        {
            UpdateVisualScale();
            _lastCamHeight = _cam.transform.position.y;
        }
        SyncCameraToUI();
    }

    private void OnEnable()
    {
        // When the tab is clicked/enabled, turn the camera on and snap it to the UI
        if (_cam != null)
        {
            _cam.gameObject.SetActive(true);
            SyncCameraToUI();
            Camera.main.clearFlags = CameraClearFlags.Depth; // Hacky set main cam to depth so sat cam can render under the UI buttons and not be cleared by main cam.
        }
    }

    private void OnDisable()
    {
        // When switching to another tab, turn the camera off so it doesn't render over other screens
        if (_cam != null)
        {
            _cam.gameObject.SetActive(false);
            Camera.main.clearFlags = _mainCamClearFlags;
        }
    }

    // --- CAMERA ---
    private void SyncCameraToUI()
    {
        if (mapUIPanel == null || _cam == null || !mapUIPanel.gameObject.activeInHierarchy)
        {
            return;
        }

        Canvas parentCanvas = mapUIPanel.GetComponentInParent<Canvas>();
        Camera uiCam = (parentCanvas != null) ? parentCanvas.worldCamera : null;

        if (uiCam == null)
        {
            Debug.LogWarning("[SatMapSystem] Cannot clip: The Canvas does not have a Render Camera assigned.");
            return;
        }

        Vector3[] corners = new Vector3[4];
        mapUIPanel.GetWorldCorners(corners);

        Vector2 bottomLeft = uiCam.WorldToScreenPoint(corners[0]);
        Vector2 topRight = uiCam.WorldToScreenPoint(corners[2]);

        // Normalize based on the camera's actual pixel dimensions
        float x = bottomLeft.x / uiCam.pixelWidth;
        float y = bottomLeft.y / uiCam.pixelHeight;
        float width = (topRight.x - bottomLeft.x) / uiCam.pixelWidth;
        float height = (topRight.y - bottomLeft.y) / uiCam.pixelHeight;

        Rect newRect = new Rect(x, y, width, height);
        _cam.rect = newRect;
    }

    public void CameraFocusWaypoint(int index)
    {
        if (index < 0 || index >= _waypointMarkers.Count)
            return;
        StopAllCoroutines();
        StartCoroutine(SmoothCameraFocus(_waypointMarkers[index].transform.position));
    }

    private Vector3 _camVel = Vector3.zero;
    private float _smoothTime = 0.2f;
    private IEnumerator SmoothCameraFocus(Vector3 target)
    {
        while (true)
        {
            Vector3 currentPos = _cam.transform.position;
            Vector3 endPos = new Vector3(target.x, currentPos.y, target.z);
            float flatDistance = Vector2.Distance(new Vector2(currentPos.x, currentPos.z), new Vector2(target.x, target.z));
            if (flatDistance <= 0.01f)
                break;

            _cam.transform.position = Vector3.SmoothDamp(
                currentPos,
                endPos,
                ref _camVel,
                _smoothTime
            );
            
            yield return null;
        }

        _cam.transform.position = new Vector3(target.x, _cam.transform.position.y, target.z);
    }

    public void HighlightWaypoint(int index, bool isHighlighted)
    {
        if (index < 0 || index >= _waypointMarkers.Count)
            return;

        GameObject marker = _waypointMarkers[index];
        Renderer r = marker.GetComponent<Renderer>();

        if (isHighlighted)
        {
            r.material.SetColor("_EmissionColor", r.material.color * 1.5f);
            marker.transform.localScale = Vector3.one * GetCurrentZoomScale() * 1.5f;
        }
        else
        {
            r.material.SetColor("_EmissionColor", Color.black);
            marker.transform.localScale = Vector3.one * GetCurrentZoomScale();
        }
    }



    private float GetCurrentZoomScale()
    {
        float t = Mathf.InverseLerp(minCamHeight, maxCamHeight, _cam.transform.position.y);
        float baseScale = Mathf.Lerp(0.2f, 4.0f, t);
        return baseScale * markerScale;
    }

    private void UpdateVisualScale()
    {
        float finalScale = GetCurrentZoomScale();
        Vector3 scaleVec = Vector3.one * finalScale;

        if (_roverMarker != null) 
            _roverMarker.transform.localScale = scaleVec;

        foreach (var marker in _waypointMarkers)
        {
            if (marker != null) marker.transform.localScale = scaleVec;
        }

        if (_pathRenderer != null)
        {
            _pathRenderer.startWidth = finalScale * 0.6f;
            _pathRenderer.endWidth = finalScale * 0.6f;
        }
        _previousMarkerScale = markerScale;
    }


    // --- MAP GENERATION ---
    void GenerateMap()
    {
        if (mapOrigin == null)
        {
            mapOrigin = new GameObject("MapOrigin");
        }
        GameObject origin = mapOrigin;
        //origin.transform.parent = this.transform;
        origin.transform.localPosition = Vector3.zero;

        for (int x = minX; x <= maxX; x++)
        {
            for (int y = minY; y <= maxY; y++)
            {
                int currentX = x;
                int currentY = y;

                _loader.LoadTile(x, y, zoomLevel, (texture) => { CreateTileObject(currentX, currentY, texture, origin.transform); });
            }
        }
    }

    void CreateTileObject(int x, int y, Texture2D tex, Transform parent)
    {
        GameObject tile = GameObject.CreatePrimitive(PrimitiveType.Quad);
        tile.name = $"({x}, {y})";
        tile.transform.parent = parent;

        // Position on X/Z Plane (Ground)
        // Map Y (North/South) corresponds to Unity Z. 
        // In Tile grids, Positive Y is South (down). In Unity, Positive Z is North.
        // We map Tile Y to Unity -Z.
        tile.transform.localPosition = new Vector3(x * tileSize, 0, -y * tileSize);

        // Rotate Quad to lay flat
        tile.transform.localRotation = Quaternion.Euler(90, 0, 0);
        tile.transform.localScale = Vector3.one * tileSize;

        // Apply Texture
        Renderer r = tile.GetComponent<Renderer>();
        r.material = baseMaterial != null ? baseMaterial : new Material(Shader.Find("Unlit/Texture"));
        r.material.mainTexture = tex;
        r.material.color = Color.red;   // for debugging

        Destroy(tile.GetComponent<Collider>());
    }

    // --- Mission Visualization ---
    public GameObject SpawnMarkerAtGPS(double lat, double lon, string markerName = "GPS_Marker", Transform parent = null, GameObject markerPrefabOverride = null)
    {
        Vector3 pos = GetUnityPositionFromGPS(lat, lon);
        pos.y = 0.1f;
        GameObject marker =  markerPrefabOverride != null ? Instantiate(markerPrefabOverride) 
            : markerPrefab != null ? Instantiate(markerPrefab) : GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = markerName;
        marker.transform.SetParent(parent ?? this.transform);
        marker.transform.localPosition = pos;
        return marker;
    }

    public void RenderWaypoints()
    {
        // Clean up markers if we deleted points
        while (_waypointMarkers.Count > currentMission.waypoints.Count)
        {
            int lastIndex = _waypointMarkers.Count - 1;
            Destroy(_waypointMarkers[lastIndex]);
            _waypointMarkers.RemoveAt(lastIndex);
        }

        _pathRenderer.positionCount = currentMission.waypoints.Count;

        for (int i = 0; i < currentMission.waypoints.Count; i++)
        {
            Vector3 worldPos = GetUnityPositionFromGPS(currentMission.waypoints[i].latitude, currentMission.waypoints[i].longitude);
            worldPos.y = 0.1f;
            Vector3 linePos = worldPos;
            linePos.y = 0.05f;
            _pathRenderer.SetPosition(i, worldPos);

            if (i >= _waypointMarkers.Count) {
                GameObject marker = markerPrefab != null ? Instantiate(markerPrefab) : GameObject.CreatePrimitive(PrimitiveType.Sphere);
                marker.transform.SetParent(_markers.transform);
                marker.AddComponent<WaypointHoverEffect>();
                _waypointMarkers.Add(marker);
            }

            _waypointMarkers[i].name = $"WP_{i}";
            _waypointMarkers[i].transform.position = worldPos;
            _waypointMarkers[i].transform.localScale = Vector3.one * GetCurrentZoomScale();

            SpriteRenderer sr = _waypointMarkers[i].GetComponentInChildren<SpriteRenderer>();
            if (sr != null)
            {
                if (i == 0) sr.sprite = startMarker;
                else if (i == currentMission.waypoints.Count - 1 && currentMission.waypoints.Count > 1) sr.sprite = goalMarker;
                else sr.sprite = pathMarker;

                Color baseColor = waypointColor;
                if (i == 0) baseColor = startColor;
                else if (i == currentMission.waypoints.Count - 1 && currentMission.waypoints.Count > 1) baseColor = goalColor;

                sr.color = baseColor;
                sr.material = new Material(sr.sharedMaterial);
                sr.material.SetColor("_Color", baseColor);

                WaypointHoverEffect hover = _waypointMarkers[i].GetComponent<WaypointHoverEffect>();
                hover.SetBaseColor(baseColor, i);

                // if (_waypointMarkers[i].TryGetComponent(out BoxCollider box)) {
                //     Vector3 spriteSize = sr.sprite.bounds.size;
                //     box.size = new Vector3(spriteSize.x, spriteSize.y, 0.2f);
                //     box.center = sr.sprite.bounds.center;
                // }

                if (_waypointMarkers[i].TryGetComponent(out BoxCollider box))
                {
                    SpriteRenderer mainSr = _waypointMarkers[i].GetComponent<SpriteRenderer>();
                    
                    Bounds combined = new Bounds(mainSr.sprite.bounds.center, mainSr.sprite.bounds.size);

                    if (_waypointMarkers[i].transform.childCount > 0 && 
                        _waypointMarkers[i].transform.GetChild(0).TryGetComponent(out SpriteRenderer circleSr) && 
                        circleSr.sprite != null)
                    {
                        Vector3 childLocalCenter = _waypointMarkers[i].transform.InverseTransformPoint(
                            circleSr.transform.TransformPoint(circleSr.sprite.bounds.center)
                        );
                        Vector3 childLocalSize = _waypointMarkers[i].transform.InverseTransformVector(circleSr.bounds.size);
                        childLocalSize = new Vector3(Mathf.Abs(childLocalSize.x), Mathf.Abs(childLocalSize.y), Mathf.Abs(childLocalSize.z));
                        combined.Encapsulate(new Bounds(childLocalCenter, childLocalSize));
                    }

                    box.size = new Vector3(combined.size.x, combined.size.y, 0.2f);
                    box.center = combined.center;
                }
            }

            // Renderer r = _waypointMarkers[i].GetComponent<Renderer>();
            // r.material.SetColor("_EmissionColor", Color.black);
            // if (i == 0) r.material.color = Color.green;
            // else if (i == currentMission.waypoints.Count - 1) r.material.color = Color.red;
            // else r.material.color = waypointColor;
        }

        _lastWaypointCount = currentMission.waypoints.Count;
        OnWaypointsChanged?.Invoke();
    }

    // -- UI Helpers ---

    public MissionConfig.SearchObject GetSearchObject() { return currentMission.searchObject; }
    public IReadOnlyList<Waypoint> GetWaypoints() { return currentMission.waypoints; }
    public int GetWaypointCount() { return currentMission.waypoints.Count; }

    public void AddWaypoint(double lat, double lon)
    {
        currentMission.waypoints.Add(new Waypoint(lat, lon));
        RenderWaypoints();
    }

    public void InsertWaypointAtIndex(int index, double lat, double lon)
    {
        if (index < 0 || index > currentMission.waypoints.Count) return;
        currentMission.waypoints.Insert(index, new Waypoint(lat, lon));
        RenderWaypoints();
    }

    public void UpdateWaypointAtIndex(int index, double newLat, double newLon)
    {
        if (index < 0 || index >= currentMission.waypoints.Count) return;
        currentMission.waypoints[index] = new Waypoint(newLat, newLon);
        RenderWaypoints();
    }

    public void RemoveWaypointAtIndex(int index)
    {
        if (index < 0 || index >= currentMission.waypoints.Count) return;
        currentMission.waypoints.RemoveAt(index);
        RenderWaypoints();
    }


    // --- Handle Inputs --- 
    private bool IsMouseOverMap()
    {
        if (mapUIPanel == null) return false;

        Canvas parentCanvas = mapUIPanel.GetComponentInParent<Canvas>();
        Camera uiCam = (parentCanvas != null) ? parentCanvas.worldCamera : null;

        return RectTransformUtility.RectangleContainsScreenPoint(mapUIPanel, Input.mousePosition, uiCam);
    }

    void HandleInteraction()
    {
        // Block if mouse is outside the UI frame or over UI element
        if (!IsMouseOverMap() || (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject()))
            return;

        Ray ray = _cam.ScreenPointToRay(Input.mousePosition);
        Plane groundPlane = new Plane(Vector3.up, Vector3.zero); // Infinite ground plane at Y=0

        // START DRAG OR ADD POINT (Left Click)
        if (Input.GetMouseButtonDown(0))
        {
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                int index = _waypointMarkers.IndexOf(hit.collider.gameObject);
                if (index != -1)
                {
                    _draggedMarker = hit.collider.gameObject;
                    _draggedIndex = index;
                    if (groundPlane.Raycast(ray, out float enterDown)) {
                        Vector3 hitPoint = ray.GetPoint(enterDown);
                        _dragOffset = _draggedMarker.transform.position - hitPoint;
                    }
                    return; // Exit early so we don't place a new waypoint under the one we just grabbed
                }
            }

            // ADD NEW POINT (insert if clicked near a path segment)
            if (groundPlane.Raycast(ray, out float enter))
            {
                Vector3 hitPoint = ray.GetPoint(enter);
                int insertIndex = GetIndexOnLineSegment(hitPoint);
                GetGPSFromUnityPosition(hitPoint, out double lat, out double lon);
                if (insertIndex != -1)
                {
                    InsertWaypointAtIndex(insertIndex, lat, lon);
                    Debug.Log($"<color=cyan>Unity({hitPoint}) => GPS({lat:F7}, {lon:F7})</color>");
                }
                else
                {
                    AddWaypoint(lat, lon);
                    Debug.Log($"<color=cyan>Unity({hitPoint}) => GPS({lat:F7}, {lon:F7})</color>");
                }

                return;
            }
        }

        // PROCESSING DRAG
        if (Input.GetMouseButton(0) && _draggedMarker != null)
        {
            if (groundPlane.Raycast(ray, out float enter))
            {
                Vector3 newWorldPos = ray.GetPoint(enter);
                Vector3 targetPos = newWorldPos + _dragOffset;
                _draggedMarker.transform.position = new Vector3(targetPos.x, 0.1f, targetPos.z);
                _pathRenderer.SetPosition(_draggedIndex, _draggedMarker.transform.position);

                GetGPSFromUnityPosition(targetPos, out double newLat, out double newLon);
                currentMission.waypoints[_draggedIndex] = new Waypoint(newLat, newLon);
                OnWaypointDragged?.Invoke(_draggedIndex, currentMission.waypoints[_draggedIndex]);
            }
        }

        // DROP
        if (Input.GetMouseButtonUp(0))
        {
            if (_draggedMarker != null)
            {
                _draggedMarker.GetComponent<WaypointHoverEffect>()?.RemoveEffect();
                OnWaypointsChanged?.Invoke();
            }
            _draggedMarker = null;
            _draggedIndex = -1;
        }

        // REMOVE POINT (Right Click)
        if (Input.GetMouseButtonDown(1))
        {
            if (Physics.Raycast(ray, out RaycastHit hit))
            {
                int index = _waypointMarkers.IndexOf(hit.collider.gameObject);
                if (index != -1)
                {
                    currentMission.waypoints.RemoveAt(index);
                    Debug.Log($"Removed Waypoint {index}");
                    OnWaypointsChanged?.Invoke();
                }
            }
        }
    }

    void HandleCameraInput()
    {
        // Block panning/zooming if mouse is outside the UI frame or over a UI element
        if (!IsMouseOverMap() || (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject()))
            return;

        // WASD / Arrow Key Panning
        float h = Input.GetAxis("Horizontal");
        float v = Input.GetAxis("Vertical");
        if (h != 0 || v != 0)
        {
            float heightMult = _cam.transform.position.y / 10.0f;
            Vector3 kbMove = new Vector3(h, 0, v) * (panSpeed * 10f) * heightMult * Time.deltaTime;
            kbMove = Quaternion.Euler(0, _cam.transform.eulerAngles.y, 0) * kbMove;
            _cam.transform.Translate(kbMove, Space.World);
        }

        // Panning (Middle Mouse Drag)
        if (Input.GetMouseButtonDown(2)) _lastMousePos = Input.mousePosition;

        if (Input.GetMouseButton(2))
        {
            Vector3 delta = Input.mousePosition - _lastMousePos;

            // Adjust pan speed based on height (pan faster when higher up)
            float heightMult = _cam.transform.position.y / 10.0f;
            Vector3 move = new Vector3(-delta.x, 0, -delta.y) * panSpeed * heightMult * Time.deltaTime;

            // Transform movement relative to camera rotation
            move = Quaternion.Euler(0, _cam.transform.eulerAngles.y, 0) * move;

            _cam.transform.Translate(move, Space.World);
            _lastMousePos = Input.mousePosition;
        }

        // Zooming (Scroll Wheel)
        float scroll = Input.mouseScrollDelta.y;
        if (scroll != 0)
        {
            Vector3 zoomDir = _cam.transform.forward * scroll * zoomSensitivity;
            Vector3 newPos = _cam.transform.position + zoomDir;

            // Clamp Height
            if (newPos.y < minCamHeight) newPos = _cam.transform.position + (Vector3.down * (_cam.transform.position.y - minCamHeight));
            if (newPos.y > maxCamHeight) newPos.y = maxCamHeight; // Simple cap

            _cam.transform.position = newPos;
        }
    }


    // --- MATH UTILITIES ---

    private int GetIndexOnLineSegment(Vector3 clickPoint)
    {
        if (currentMission.waypoints.Count < 2) return -1;

        // Scale the threshold by zoom so it's always easy to click
        float effectiveThreshold = 2.0f * (GetCurrentZoomScale() * 0.5f);

        for (int i = 0; i < currentMission.waypoints.Count - 1; i++)
        {
            Vector3 p1 = _waypointMarkers[i].transform.position;
            Vector3 p2 = _waypointMarkers[i + 1].transform.position;

            // Calculate distance from clickPoint to the segment p1-p2
            float distance = DistanceToSegment(clickPoint, p1, p2);

            if (distance < effectiveThreshold)
            {
                return i + 1; // Insert at the index of the second point in the segment
            }
        }

        return -1;
    }

    private float DistanceToSegment(Vector3 p, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a;
        Vector3 ap = p - a;
        float t = Vector3.Dot(ap, ab) / Vector3.Dot(ab, ab);
        t = Mathf.Clamp01(t);
        Vector3 closestPoint = a + t * ab;
        return Vector3.Distance(p, closestPoint);
    }

    public void GetGPSFromUnityPosition(Vector3 pos, out double lat, out double lon)
    {
        // Normalize Unity units to Tile units
        double xOffsetTiles = pos.x / tileSize;
        double yOffsetTiles = pos.z / tileSize; // Unity Z is Map Y (North/South)

        // Apply to Origin Global Coordinate
        // Tile X increases East (Unity +X)
        double targetGlobalX = _originGlobalTileX + xOffsetTiles;

        // Tile Y increases SOUTH. Unity Z increases NORTH.
        double targetGlobalY = _originGlobalTileY - yOffsetTiles;

        // Convert back to Lat/Lon
        lon = TileXToLongitude(targetGlobalX, zoomLevel);
        lat = TileYToLatitude(targetGlobalY, zoomLevel);
    }

    public Vector3 GetUnityPositionFromGPS(double lat, double lon)
    {
        // Convert Global GPS to Global Web Mercator tile coordinates
        double targetGlobalX = LongitudeToTileX(lon, zoomLevel);
        double targetGlobalY = LatitudeToTileY(lat, zoomLevel);

        // Subtract the origin to get the offset in tiles
        double offsetX = targetGlobalX - _originGlobalTileX;
        double offsetY = targetGlobalY - _originGlobalTileY;

        // Scale by tileSize to get Unity Units
        float x = (float)(offsetX * tileSize);
        float z = (float)(-offsetY * tileSize);
        return new Vector3(x, 0, z);
    }

    // Standard Web Mercator Formulas
    private static double LongitudeToTileX(double lon, int zoom)
    {
        return (lon + 180.0) / 360.0 * Math.Pow(2.0, zoom);
    }

    private static double LatitudeToTileY(double lat, int zoom)
    {
        double latRad = lat * Math.PI / 180.0;
        return (1.0 - Math.Log(Math.Tan(latRad) + (1.0 / Math.Cos(latRad))) / Math.PI) / 2.0 * Math.Pow(2.0, zoom);
    }

    private static double TileXToLongitude(double tileX, int zoom)
    {
        return tileX / Math.Pow(2.0, zoom) * 360.0 - 180.0;
    }

    private static double TileYToLatitude(double tileY, int zoom)
    {
        double tileYNorm = Math.PI - 2.0 * Math.PI * tileY / Math.Pow(2.0, zoom);
        return 180.0 / Math.PI * Math.Atan(0.5 * (Math.Exp(tileYNorm) - Math.Exp(-tileYNorm)));
    }


    
    public void SetSearchObject(MissionConfig.SearchObject newSearchObject) 
    {
        buttonSearchObject = newSearchObject;
        currentMission.searchObject = buttonSearchObject;
        Debug.Log("Search object set to: " + currentMission.searchObject);
    }

    void OnValidate()
    {
        // This ensures the markers update immediately when you slide the slider in the Editor
        if (Application.isPlaying && _cam != null) UpdateVisualScale();

        if (currentMission != null)
        {
            currentMission.searchPattern = inspectorSearchPattern;
            currentMission.searchObject = buttonSearchObject;
            currentMission.searchParam1 = inspectorSearchParam1;
            currentMission.searchParam2 = inspectorSearchParam2;
        }
    }
}

