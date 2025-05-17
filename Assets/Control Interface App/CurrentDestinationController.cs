using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurrentDestinationController : MonoBehaviour
{
    public static CurrentDestinationController inst;
    [SerializeField] GpsLocation currentTarget;
    [SerializeField] List<Coordinate> waypoints = new();
    [SerializeField] int waypointIndex = 0;
    [SerializeField] float distanceCutoff = .1f;
    [SerializeField] bool isInReverse;

    struct Coordinate
    {
        public double lat;
        public double lon;
    }

    private void Awake()
    {
        inst = this;
    }

    public void ReturnToStart()
    {
        isInReverse = true;
        if (waypointIndex == 0) return;
        waypointIndex--;
        SendNextWaypoint(true);
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
        script.SetActive(); // makes the Go button turn red and say stop
        currentTarget = script;
        isInReverse = false;
        waypoints.Clear();
        waypointIndex = 0;
        foreach (var x in script.waypoints) {
            var coords = MapController.instance.GetLatLonFromWorldPosition(x.transform.position);
            waypoints.Add(new Coordinate() { 
                lat = Math.Round(coords[0], 6),
                lon = Math.Round(coords[1], 6) });
        }
        var finalDestination = new Coordinate() {
            lat = Math.Round(double.Parse(script.lat.text), 6),
            lon = Math.Round(double.Parse(script.lon.text), 6)
        };
        waypoints.Add(finalDestination);
        SendNextWaypoint(true);
        StatusIndicator.instance.SetIndicator(Status.Activated, script);
        // this should be done on callback from the rover
    }

    void SendNextWaypoint(bool turnFirst = false) {
        string message = $"autonomous/auton_control;GOTO;{waypoints[waypointIndex].lat};{waypoints[waypointIndex].lon};{turnFirst}";
        TcpController.inst.Publish(message);
    }

    public void Stop(GpsLocation script)
    {
        script.SetInactive();
        string message = $"autonomous/auton_control;STOP;{script.lat.text};{script.lon.text}";
        TcpController.inst.Publish(message);

        StatusIndicator.instance.SetIndicator(Status.NotActivated, script);
        currentTarget = null;
        // this should be done on callback from the rover
    }

    public void ReceivePositionUpdate(double lat, double lon) {
        if (currentTarget == null) return;

        var worldPos = MapController.instance.GetWorldPosition(lat, lon);
        var targetWorldPos = waypoints[waypointIndex];
        Vector2 targetVect = MapController.instance.GetWorldPosition(targetWorldPos.lat, targetWorldPos.lon);
        var dist = Vector2.Distance(worldPos, targetVect);
        Debug.Log("Distance to target: " + dist);

        if (dist < distanceCutoff) {
            if (isInReverse) waypointIndex--;
            else waypointIndex++;

            if(waypointIndex >= waypoints.Count) {
                Debug.Log("Reached destination");
                Stop(currentTarget);
            }
            else if(waypointIndex < 0)
            {
                Debug.Log("Returned");
                Stop(currentTarget);
            }
            else {
                Debug.Log("Send next waypoint");
                SendNextWaypoint();
            }
        }
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
