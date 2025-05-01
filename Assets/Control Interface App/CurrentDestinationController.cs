using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurrentDestinationController : MonoBehaviour
{
    public static CurrentDestinationController inst;
    GpsLocation currentTarget;

    private void Awake()
    {
        inst = this;
    }

    public void ClickBtn(GpsLocation script)
    {
        if(currentTarget == script)
        {
            currentTarget = null;
            Stop(script);
        }
        else
        {
            if(currentTarget != null) currentTarget.SetInactive();
            currentTarget = script;
            SetDestination(script);
        }
    }

    public void SetDestination(GpsLocation script)
    {
        script.SetActive();
        //string message = $"autonomous/auton_control;GOTO;{script.lat.text};{script.lon.text}";
        WaypointList waypointList = new(){
            list = new()
        };
        foreach(var x in script.waypoints){
            var coords = MapController.instance.GetLatLonFromWorldPosition(x.transform.position);
            Waypoint point = new(){
                lat = coords[0].ToString("F5"),
                lon = coords[1].ToString("F5")
            };
            waypointList.list.Add(point);
        }
        Waypoint finalPoint = new()
        {
            lat = script.lat.text,
            lon = script.lon.text
        };
        waypointList.list.Add(finalPoint);
        string json = JsonUtility.ToJson(waypointList);
        json = "autonomous/auton_control;GOTO;" + json;
        Debug.Log("json: " +json);

        //string message = $"autonomous/auton_control;FIND;ARUCO";
        TcpController.inst.Publish(json);
        StatusIndicator.instance.SetIndicator(Status.Activated, script);
        // this should be done on callback from the rover

        //MapController.instance.SetLinePosition(script.iconObject.transform.position);
    }

    public void Stop(GpsLocation script)
    {
        script.SetInactive();
        string message = $"autonomous/auton_control;STOP;{script.lat.text};{script.lon.text}";
        TcpController.inst.Publish(message);

        StatusIndicator.instance.SetIndicator(Status.NotActivated, script);
        // this should be done on callback from the rover
    }

    [System.Serializable]
    class WaypointList {
        public List<Waypoint> list;
    }

    [System.Serializable]
    class Waypoint{
        public string lat;
        public string lon;
    }
}
