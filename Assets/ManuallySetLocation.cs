using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ManuallySetLocation : MonoBehaviour
{
    [SerializeField] TextMeshProUGUI btnText;
    bool setting;
    public void Set()
    {
        btnText.text = "SETTING";
        setting = true;
    }

    private void Update()
    {
        if(setting && Input.GetMouseButtonDown(0))
        {
            setting = false;
            Vector2 worldPos = WaypointAdder.inst.GetWorldPos();
            RoverIconController.inst.roverIcon.position = worldPos;
            btnText.text = "Set Location";
            RoverTrail.inst.ResetTrail();
            var latlon = MapController.instance.GetLatLonFromWorldPosition(worldPos);
            string msg = "autonomous/manually_set_position;" + latlon[0] + ";" + latlon[1];
            TcpController.inst.Publish(msg);
        }
    }
}
