using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MapController : MonoBehaviour
{
    [SerializeField] SpriteRenderer sprite;
    [SerializeField] float bottomLeftCornerLat;
    [SerializeField] float bottomLeftCornerLong;
    [SerializeField] float topRightCornerLat;
    [SerializeField] float topRightCornerLong;
    [SerializeField] GameObject icon;
    // Start is called before the first frame update
    void Start()
    {
        float width = sprite.bounds.size.x;
        float height = sprite.bounds.size.y;
        Debug.Log("Sprite Width: " + width);
    }


    public void AddCoordinate(float lat, float lon){
        // Vector2 position = new();
        // Transform parent = null;
        // GameObject newObject = Instantiate(icon, position, Quaternion.identity);
        // newObject.transform.SetParent(icon, true);  // 'true' keeps the current world position
    }
}
