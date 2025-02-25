using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class MapController : MonoBehaviour
{
    public static MapController instance;
    [SerializeField] SpriteRenderer mapSprite;
    [SerializeField] Transform roverIcon;
    [SerializeField] double bottomLeftCornerLat;
    [SerializeField] double bottomLeftCornerLong;
    [SerializeField] double topRightCornerLat;
    [SerializeField] double topRightCornerLong;
    [SerializeField] double scaleX; //world units per degree
    [SerializeField] double scaleY;
    [SerializeField] float width;
    [SerializeField] float height;
    [SerializeField] Vector2 lineTarget;

    public GameObject icon;
    public Transform iconsParent;
    [SerializeField] GameObject newListingMenu;
    [SerializeField] TMP_InputField descText;
    [SerializeField] TMP_InputField latText;
    [SerializeField] TMP_InputField longText;
    [SerializeField] Toggle homeIconToggle;
    [SerializeField] LineRenderer lineRenderer;
    [SerializeField] Sprite homeIcon;

    void Awake()
    {
        instance = this;

        width = mapSprite.bounds.size.x;
        height = mapSprite.bounds.size.y;

        double widthInDegrees = topRightCornerLong - bottomLeftCornerLong;
        scaleX = (double)width / widthInDegrees;

        double heightInDegrees = topRightCornerLat - bottomLeftCornerLat;
        scaleY = (double)height / heightInDegrees;
        //AddCoordinate(44.565631d, -123.276036d);
        lineTarget = Vector3.zero;
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
        GameObject newObject = Instantiate(icon, position, Quaternion.identity);
        newObject.transform.SetParent(parent, true);  // 'true' keeps the current world position
        PreloadedIcon script = newObject.GetComponent<PreloadedIcon>();
        script.latitude = lat;
        script.longitude = lon;
        script.description = descText.text;
        if (homeIconToggle.isOn)
        {
            SpriteRenderer sr = newObject.transform.GetChild(0).GetComponent<SpriteRenderer>();
            sr.sprite = homeIcon;
            sr.color = new Color(12 / 255f, 1f, 0f);
            newObject.transform.GetChild(0).localPosition = new Vector3();
        }
        newListingMenu.SetActive(false);
        CameraControl.inst.RescaleIcons();
    }

    

    public void MoveIcon(Transform obj, double lat, double lon)
    {
        Vector2 position = GetWorldPosition(lat, lon);
        obj.position = position;
    }

    public Vector2 GetWorldPosition(double lat, double lon)
    {
        double xFromBLCornerInDeg = lon - bottomLeftCornerLong;
        double yFromBLCornerInDeg = lat - bottomLeftCornerLat;

        float xFromBLCornerInM = (float)(xFromBLCornerInDeg * scaleX);
        float yFromBLCornerInM = (float)(yFromBLCornerInDeg * scaleY);

        float worldUnitsX = xFromBLCornerInM - width / 2f;
        float worldUnitsY = yFromBLCornerInM - height / 2f;
        return new Vector2(worldUnitsX, worldUnitsY);
    }

    public List<double> GetLatLonFromWorldPosition(Vector2 worldPos)
    {
        // Convert world coordinates to meters
        float xFromBLCornerInM = worldPos.x + width / 2f;
        float yFromBLCornerInM = worldPos.y + height / 2f;

        // Convert meters to degrees
        double xFromBLCornerInDeg = xFromBLCornerInM / scaleX;
        double yFromBLCornerInDeg = yFromBLCornerInM / scaleY;

        // Compute latitude and longitude
        double lon = bottomLeftCornerLong + xFromBLCornerInDeg;
        double lat = bottomLeftCornerLat + yFromBLCornerInDeg;

        List<double> results = new List<double>() { lat, lon };
        return results;
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
        lineRenderer.enabled = false;
    }
}
