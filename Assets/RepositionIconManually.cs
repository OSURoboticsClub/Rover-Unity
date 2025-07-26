using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// adds functionality to reposition an icon to the mouse position by pressing M

public class RepositionIconManually : MonoBehaviour
{

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.M)) {
            var script = GetComponent<SelectDestination>();
            if (!script.isSelected) return;
            var mouseWorldPos = GetMapMousePosition.inst.GetWorldPositionFromMouse();
            //Debug.Log(mouseWorldPos);
            if (mouseWorldPos == null) return;
            GpsLocation location = GetComponent<GpsLocation>();
            Transform iconTransform = location.iconObject.transform;
            iconTransform.position = new Vector3(mouseWorldPos.Value.x, mouseWorldPos.Value.y, iconTransform.position.z);
            var coords = MapController.instance.GetLatLonFromWorldPosition(new Vector2(mouseWorldPos.Value.x, mouseWorldPos.Value.y));
            location.lat.text = coords[0].ToString().Length >= 11 ? coords[0].ToString().Substring(0, 11) : coords[0].ToString();
            location.lon.text = coords[1].ToString().Length >= 11 ? coords[1].ToString().Substring(0, 11) : coords[1].ToString();
            var scr = location.iconObject.GetComponent<PreloadedIcon>();
            scr.latitude = coords[0];
            scr.longitude = coords[1];
        }
    }
}
