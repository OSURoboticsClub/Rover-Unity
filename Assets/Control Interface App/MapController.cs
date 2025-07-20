using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class MapController : MonoBehaviour
{
    public static MapController instance;
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
    [SerializeField] Sprite hammerIcon;
    [SerializeField] Sprite arucoIcon;
    [SerializeField] TMP_Dropdown iconDropdown;
    [SerializeField] Transform mapsParent;
    MapData currMap;

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

        

        SpriteRenderer sr = iconOnMap.transform.GetChild(0).GetComponent<SpriteRenderer>();
        Sprite sprite = gnssIcon;
        if (iconDropdown.value == 1) sprite = arucoIcon;
        else if (iconDropdown.value == 2) sprite = hammerIcon;
        else if (iconDropdown.value == 3) sprite = homeIcon;
        sr.sprite = sprite;

        newListingMenu.SetActive(false);
        CameraControl.inst.RescaleIcons();

        // the row in the table for the location is created in Start() in PreloadedIcon
    }

    public void MoveIcon(Transform obj, double lat, double lon)
    {
        Vector2 position = GetWorldPosition(lat, lon);
        obj.position = position;
    }

    public Vector2 GetWorldPosition(double lat, double lon) {
        const double centerLat = 44.56479;
        const double centerLon = -123.27381;
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


    public List<double> GetLatLonFromWorldPosition(Vector2 worldPos)
    {
        const double centerLat = 44.56479;
        const double centerLon = -123.27381;
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
}
