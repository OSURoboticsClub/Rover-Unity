using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaypointAdder : MonoBehaviour
{
    public static WaypointAdder inst;
    public static bool inAddingMode = false;
    public GpsLocation currentLocationBeingModified = null;
    public Texture2D cursorTexture; // Assign your cursor image in the Inspector
    public Vector2 hotspot = Vector2.zero; // Adjust if needed
    public float t;

    public bool isOverMap = false;
    private void Awake()
    {
        inst = this;
    }
    public void OpenAddingMode(GpsLocation location)
    {
        inAddingMode = true;
        currentLocationBeingModified = location;
    }

    public void CloseAddingMode()
    {
        inAddingMode = false;
        currentLocationBeingModified = null;
        Cursor.SetCursor(null, Vector2.zero, CursorMode.Auto);
    }

    void ChangeCursor(bool isNowOverMap)
    {
        if (isNowOverMap == isOverMap) return;
        isOverMap = isNowOverMap;
        if (isOverMap)
        {
            Cursor.SetCursor(cursorTexture, hotspot, CursorMode.ForceSoftware);
        }
        else
        {
            Cursor.SetCursor(null, Vector2.zero, CursorMode.Auto);
            return;
        }
    }

    private void Update()
    {
        if (!inAddingMode) return;

        bool isNowOverMap = CameraControl.inst.IsMouseOverMap();
        ChangeCursor(isNowOverMap);

        if(Input.GetMouseButton(0))
        {
            t += Time.deltaTime;
        }
        else if(t > 0)
        {
            if(t < 0.15f)
            {
                Debug.Log("Clicked");
            }
            t = 0;
        }
    }
}
