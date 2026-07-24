using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class MapController : MonoBehaviour
{
    public static MapController instance;
    //[SerializeField] Camera cam;
    [SerializeField] Transform roverIcon;
    public Vector2 lineTarget;

    public GameObject icon;
    public Transform iconsParent;
    [SerializeField] GameObject newListingMenu;
    [SerializeField] TMP_InputField descText;
    [SerializeField] TMP_InputField latText;
    [SerializeField] TMP_InputField longText;
    [SerializeField] LineRenderer lineRenderer;
    [SerializeField] Sprite gnssIcon;
    [SerializeField] Sprite homeIcon;
    [SerializeField] Sprite objectIcon;
    [SerializeField] Sprite arucoIcon;
    [SerializeField] TMP_Dropdown iconDropdown;
    [SerializeField] TMP_Dropdown objectsDropdown;
    [SerializeField] GameObject objectDetectionPanel;
    [SerializeField] Transform mapsParent;
    MapData currMap;

    public ObjectType objectType;
    public enum IconType {gnssIcon, arucoIcon, objectIcon, homeIcon}
    public IconType iconType;
    public ObjectDetectionImageDisplay imageDisplay;
    private bool panelWasOpen = false;
    private ObjectButtonHandler _currentSelectedObject = null;

    void Awake()
    {
        instance = this;

        //width = mapSprite.bounds.size.x;
        //height = mapSprite.bounds.size.y;

        //double widthInDegrees = topRightCornerLong - bottomLeftCornerLong;
        //scaleX = (double)width / widthInDegrees;

        //double heightInDegrees = topRightCornerLat - bottomLeftCornerLat;
        //scaleY = (double)height / heightInDegrees;
        //AddCoordinate(44.565631d, -123.276036d);
        lineTarget = Vector3.zero;

        foreach(Transform child in mapsParent) {
            if (!child.gameObject.activeSelf) continue;
            currMap = child.GetComponent<MapData>();
        }
    }

    private void Update()
    {
        if(lineTarget != Vector2.zero)
        {
            SetLinePosition();
        }
    }

    public void AddCoordinateFromUIMenu()
    {
        bool succLat = double.TryParse(latText.text, out double lat);
        bool succlon = double.TryParse(longText.text, out double lon);
        if (!succLat || !succlon) return;

        Vector2 position = GetWorldPosition(lat, lon);
        Transform parent = iconsParent;
        GameObject iconOnMap = Instantiate(icon, position, Quaternion.identity);
        iconOnMap.transform.SetParent(parent, true);  // 'true' keeps the current world position
        PreloadedIcon script = iconOnMap.GetComponent<PreloadedIcon>();
        script.latitude = lat;
        script.longitude = lon;
        script.description = descText.text;

        script.objectType = (ObjectType)objectsDropdown.value;

        SpriteRenderer sr = iconOnMap.transform.GetChild(0).GetComponent<SpriteRenderer>();
        Sprite sprite = gnssIcon;
        if (iconDropdown.value == 1) sprite = arucoIcon;
        else if (iconDropdown.value == 2) sprite = objectIcon;
        else if (iconDropdown.value == 3) sprite = homeIcon;
        sr.sprite = sprite;

        newListingMenu.SetActive(false);
        CameraControl.inst.RescaleIcons();

        CheckObjectDetectionPanel();
        // the row in the table for the location is created in Start() in PreloadedIcon
    }

     // Accept parameters directly so no parsing is needed
	public void AddCoordinateClick(double lat, double lon, string description, int iconTypeIndex)
	{
	    // Use the parameters passed into the function
	    Vector2 position = GetWorldPosition(lat, lon);
	    Transform parent = iconsParent;
	    
	    GameObject iconOnMap = Instantiate(icon, position, Quaternion.identity);
	    iconOnMap.transform.SetParent(parent, true); // Change back to true

	    PreloadedIcon script = iconOnMap.GetComponent<PreloadedIcon>();
	    script.latitude = lat;
	    script.longitude = lon;
	    script.description = description;

	    SpriteRenderer sr = iconOnMap.transform.GetChild(0).GetComponent<SpriteRenderer>();
	    
	    // Assign sprite based on the passed index
	    Sprite sprite = gnssIcon;
	    if (iconTypeIndex == 1) sprite = arucoIcon;
	    else if (iconTypeIndex == 2) sprite = objectIcon;
	    else if (iconTypeIndex == 3) sprite = homeIcon;
	    sr.sprite = sprite;

	    // Optional: Only hide menu if this was called from the UI
	    if(newListingMenu.activeSelf) newListingMenu.SetActive(false);
	    
	    CameraControl.inst.RescaleIcons();
	}

    public void MoveIcon(Transform obj, double lat, double lon)
    {
        Vector2 position = GetWorldPosition(lat, lon);
        obj.position = position;
    }

    public Vector2 GetWorldPosition(double lat, double lon) {
        double centerLat = currMap.lat;
        double centerLon = currMap.lon;
        const int zoom = 19;
        const float unityUnitsPerTile = 1.0f;

        double iconTileX = LonToTileX(lon, zoom);
        double iconTileY = LatToTileY(lat, zoom);
        double centerTileX = LonToTileX(centerLon, zoom);
        double centerTileY = LatToTileY(centerLat, zoom);

        // Subtract 0.5 to align tile centers to Unity (0,0)
        double deltaX = (iconTileX - centerTileX) * unityUnitsPerTile;
        double deltaY = (iconTileY - centerTileY) * unityUnitsPerTile;

        return new Vector2((float)deltaX, (float)-deltaY);
    }


    double LonToTileX(double lon, int zoom) {
        return (lon + 180.0) / 360.0 * (1 << zoom);
    }

    double LatToTileY(double lat, int zoom) {
        double latRad = lat * Math.PI / 180.0;
        return (1.0 - Math.Log(Math.Tan(latRad) + 1.0 / Math.Cos(latRad)) / Math.PI) / 2.0 * (1 << zoom);
    }

    double TileXToLon(double x, int zoom) {
        return x / (1 << zoom) * 360.0 - 180.0;
    }

    double TileYToLat(double y, int zoom) {
        double n = Math.PI - 2.0 * Math.PI * y / (1 << zoom);
        return 180.0 / Math.PI * Math.Atan(Math.Sinh(n));
    }


    /*public List<double> GetLatLonFromWorldPosition(Vector2 worldPos)
    {
        const double centerLat = 51.464553;
        const double centerLon = -112.7275881;
        const int zoom = 19;
        const float unityUnitsPerTile = 1.0f;

        double centerTileX = LonToTileX(centerLon, zoom);
        double centerTileY = LatToTileY(centerLat, zoom);

        // Inverse of world position to tile delta
        double iconTileX = worldPos.x / unityUnitsPerTile + centerTileX;
        double iconTileY = -worldPos.y / unityUnitsPerTile + centerTileY;

        double lon = TileXToLon(iconTileX, zoom);
        double lat = TileYToLat(iconTileY, zoom);

        return new List<double>() { lat, lon };
    }*/
    	public List<double> GetLatLonFromWorldPosition(Vector2 worldPos)
	{
	    // Use the dynamic center from your active map data (No more Canada!)
	    // We use a null check just in case currMap isn't set yet
	    double centerLat = currMap != null ? currMap.lat : 44.566;
	    double centerLon = currMap != null ? currMap.lon : -123.272;
	    
	    const int zoom = 19;
	    const float unityUnitsPerTile = 1.0f;

	    double centerTileX = LonToTileX(centerLon, zoom);
	    double centerTileY = LatToTileY(centerLat, zoom);

	    // Inverse of world position to tile delta
	    // Unity +X is East, Tile +X is East
	    // Unity +Y is North, Tile +Y is South (hence the negative worldPos.y)
	    double iconTileX = (worldPos.x / unityUnitsPerTile) + centerTileX;
	    double iconTileY = -(worldPos.y / unityUnitsPerTile) + centerTileY;

	    double lon = TileXToLon(iconTileX, zoom);
	    double lat = TileYToLat(iconTileY, zoom);

	    return new List<double>() { lat, lon };
	}
    public void ReceiveNextDestination(string msg)
    {
        var parts = msg.Split(";");
        float lat = float.Parse(parts[2]);
        float lon = float.Parse(parts[3]);
        lineTarget = GetWorldPosition(lat, lon);
        lineRenderer.enabled = true;
        //SetLinePosition();
    }

    public void SetLinePosition()
    {
        lineRenderer.enabled = true;
        Vector3 target = lineTarget;
        target.z = 0.5f;
        Vector3 roverPos = roverIcon.position;
        roverPos.z = 0.5f;

        lineRenderer.SetPosition(0, target);
        lineRenderer.SetPosition(1, roverPos);
    }

    public void SetLineScale(float scale)
    {
        lineRenderer.widthMultiplier = scale;
    }

    public void TurnOffLine()
    {
        lineTarget = Vector2.zero;
        lineRenderer.enabled = false;
    }

	public double GetCurrMapLat() => currMap != null ? currMap.lat : 44.566; // Fallback to Oregon
	public double GetCurrMapLon() => currMap != null ? currMap.lon : -123.272;

    public void CheckObjectDetectionPanel() 
    {
        IconType iconType = (IconType)iconDropdown.value;
        ObjectType objectType = (ObjectType)objectsDropdown.value;
        bool isObjectIcon = iconType == IconType.objectIcon;
        bool hasObject = objectType != ObjectType.None;
        bool showPanel = isObjectIcon && hasObject;
        objectDetectionPanel.SetActive(showPanel);

        if (showPanel && !panelWasOpen) {
            imageDisplay.Open();
        } else if (!showPanel && panelWasOpen) {
            imageDisplay.Close();
        }

        panelWasOpen = showPanel;
    }

    public void OnObjectButtonClicked(ObjectButtonHandler clickedButton)
    {
        Debug.Log($"Object button clicked: {clickedButton.objectType}");
        if (_currentSelectedObject == clickedButton) {
            // if clicking same button :: toggle false
            _currentSelectedObject.UpdateVisual(false);
            _currentSelectedObject = null;
            objectDetectionPanel.SetActive(false);
            imageDisplay?.Close();
            return;
        }

        if (_currentSelectedObject != null) {
            // if clicking new button :: toggle true
            _currentSelectedObject.UpdateVisual(false);
        }

        _currentSelectedObject = clickedButton;
        _currentSelectedObject.UpdateVisual(true);
        SatelliteMapSystem.Instance.SetSearchObject(clickedButton.searchObject);

        objectDetectionPanel.SetActive(true);
        imageDisplay?.Open();
        MissionConfig.SearchObject buttonSearchObject;
    }
}
