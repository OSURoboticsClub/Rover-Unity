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
        string message = $"autonomous/auton_control;GOTO;{script.lat.text};{script.lon.text}";
        WaypointList waypointList = new(){
            list = new()
        };
        foreach(var x in script.waypoints){
            Waypoint point = new(){
                lat = x.transform.position.x.ToString(),
                lon = x.transform.position.y.ToString()
            };
            waypointList.list.Add(point);
        }
        string json = JsonUtility.ToJson(waypointList);
        Debug.Log(json);

        //string message = $"autonomous/auton_control;FIND;ARUCO";
        //TcpController.inst.Publish(message);
        //StatusIndicator.instance.SetIndicator(Status.Activated, script);
        // this should be done on callback from the rover

        //MapController.instance.SetLinePosition(script.iconObject.transform.position);
    }

    public void Stop(GpsLocation script)
    {
        script.SetInactive();
        string message = $"autonomous/auton_control;STOP;{script.lat.text};{script.lon.text}";
        TcpController.inst.Publish(message);

        //StatusIndicator.instance.SetIndicator(Status.NotActivated, script);
        // this should be done on callback from the rover
    }

    class WaypointList{
        public List<Waypoint> list;
    }

    class Waypoint{
        public string lat;
        public string lon;
    }
}
