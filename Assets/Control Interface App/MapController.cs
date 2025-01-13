using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MapController : MonoBehaviour
{
    public static MapController instance;
    [SerializeField] SpriteRenderer sprite;
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
    // Start is called before the first frame update
    void Awake()
    {
        instance = this;

        width = sprite.bounds.size.x;
        height = sprite.bounds.size.y;

        double widthInDegrees = topRightCornerLong - bottomLeftCornerLong;
        scaleX = (double)width / widthInDegrees;

        double heightInDegrees = topRightCornerLat - bottomLeftCornerLat;
        scaleY = (double)height / heightInDegrees;
        //AddCoordinate(44.565631d, -123.276036d);
    }


    public void AddCoordinate(double lat, double lon){
        Vector2 position = GetWorldPosition(lat, lon);
        Transform parent = iconsParent;
        GameObject newObject = Instantiate(icon, position, Quaternion.identity);
        newObject.transform.SetParent(parent, true);  // 'true' keeps the current world position
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
}
