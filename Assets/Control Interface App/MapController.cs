using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

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

    [SerializeField] GameObject icon;
    [SerializeField] Transform iconsParent;
    [SerializeField] GameObject newListingMenu;
    [SerializeField] TMP_InputField descText;
    [SerializeField] TMP_InputField latText;
    [SerializeField] TMP_InputField longText;
    [SerializeField] LineRenderer lineRenderer;
    // Start is called before the first frame update
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
    }

    public void AddCoordinateFromUIMenu()
    {
        bool succLat = double.TryParse(latText.text, out double lat);
        bool succlon = double.TryParse(longText.text, out double lon);
        if (!succLat || !succlon) return;
        AddCoordinate(lat, lon, descText.text);
        newListingMenu.SetActive(false);
    }

    public void AddCoordinate(double lat, double lon, string desc){
        Vector2 position = GetWorldPosition(lat, lon);
        Transform parent = iconsParent;
        GameObject newObject = Instantiate(icon, position, Quaternion.identity);
        newObject.transform.SetParent(parent, true);  // 'true' keeps the current world position
        PreloadedIcon script = newObject.GetComponent<PreloadedIcon>();
        script.latitude = lat;
        script.longitude = lon;
        script.description = desc;
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

    public void SetLinePosition(Vector3 target)
    {
        lineRenderer.enabled = true;
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
