using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum ItemToFind
{
    none,
    hammer,
    bottle,
    aruco
}

public class CurrentDestinationController : MonoBehaviour
{
    public static CurrentDestinationController inst;
    public GpsLocation currentTarget;
    Vector2 currentVector2Target;
    [SerializeField] List<Coordinate> waypoints = new();
    [SerializeField] int waypointIndex = 0;
    [SerializeField] float distanceCutoff = .1f;
    [SerializeField] bool isInReverse;
    [SerializeField] GameObject circle;
    public ItemToFind item;
    List<GameObject> circleIcons = new();
    [SerializeField] Transform iconParent;

    public List<Vector2> squarePoints = new();
    int squarePointIndex = -1;

    public float distToTarget = 1000f;
    bool searchingForAruco = false;

    struct Coordinate
    {
        public double lat;
        public double lon;
    }

    private void Awake()
    {
        inst = this;
    }

    void Update()
    {
        if (currentTarget != null)
        {
            distToTarget = Vector2.Distance(currentTarget.iconObject.transform.position, RoverIconController.inst.roverIcon.position);
        }
        else distToTarget = 10000f;
    }

    public void ReturnToStart()
    {
        isInReverse = true;
        if (waypointIndex == 0) return;
        waypointIndex--;
        SendNextWaypoint(true);
    }

    public void ClickGoButton(GpsLocation script)
    {
        // what happens when you click Go on a location depends whether the location is
        // currently "active" as an autonomy destination

        squarePoints.Clear();
        squarePointIndex = -1;
        searchingForAruco = false;
        foreach (var x in circleIcons) {
            Destroy(x);
        }
        circleIcons.Clear();
        // use currentTarget to check if the location is active
        if (currentTarget == script)
        {
            currentTarget = null;
            Stop(script);
            MapController.instance.TurnOffLine();
            script.line.enabled = false;
        }
        else
        {
            if(currentTarget != null) currentTarget.SetInactiveUI();

            script.SetActive(); // makes the Go button turn red and say stop
            currentTarget = script;
            item = script.itemAtDestination;
            isInReverse = false;
            waypoints.Clear();
            waypointIndex = 0;
            foreach (var x in script.waypoints)
            {
                var coords = MapController.instance.GetLatLonFromWorldPosition(x.transform.position);
                waypoints.Add(new Coordinate()
                {
                    lat = Math.Round(coords[0], 6),
                    lon = Math.Round(coords[1], 6)
                });
            }
            var finalDestination = new Coordinate()
            {
                lat = Math.Round(double.Parse(script.lat.text), 6),
                lon = Math.Round(double.Parse(script.lon.text), 6)
            };
            waypoints.Add(finalDestination);
            float dist = .75f;
            Vector2 finalDestWorldPos = MapController.instance.GetWorldPosition(finalDestination.lat, finalDestination.lon);
            squarePoints.Add(new Vector2(finalDestWorldPos.x + dist, finalDestWorldPos.y - dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x - dist, finalDestWorldPos.y - dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x - dist, finalDestWorldPos.y + dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x + dist, finalDestWorldPos.y + dist));
            dist = 1.6f;
            squarePoints.Add(new Vector2(finalDestWorldPos.x + dist, finalDestWorldPos.y + dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x, finalDestWorldPos.y + dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x - dist, finalDestWorldPos.y + dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x - dist, finalDestWorldPos.y));
            squarePoints.Add(new Vector2(finalDestWorldPos.x - dist, finalDestWorldPos.y - dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x, finalDestWorldPos.y - dist));

            squarePoints.Add(new Vector2(finalDestWorldPos.x + dist, finalDestWorldPos.y - dist));
            squarePoints.Add(new Vector2(finalDestWorldPos.x + dist, finalDestWorldPos.y));
            foreach (var x in squarePoints) {
                var obj = Instantiate(circle, iconParent);
                obj.transform.position = x;
                circleIcons.Add(obj);
            }
            CameraControl.inst.RescaleIcons();
            SendNextWaypoint(true);
            StatusIndicator.instance.SetIndicator(Status.Activated, script);
            // TODO: this should be done on callback from the rover
        }
    }

    void SendNextWaypoint(bool turnFirst = false) {
        Coordinate target = waypoints[waypointIndex];
        currentVector2Target = MapController.instance.GetWorldPosition(target.lat, target.lon);
        string message = $"autonomous/auton_control;GOTO;{waypoints[waypointIndex].lat};{waypoints[waypointIndex].lon};{turnFirst}";
        var worldPos = MapController.instance.GetWorldPosition(waypoints[waypointIndex].lat, waypoints[waypointIndex].lon);
        MapController.instance.lineTarget = worldPos;

        TcpController.inst.Publish(message);
    }

    public void Stop(GpsLocation script = null, bool haveLedBlink = false)
    {
        TcpController.inst.Publish($"autonomous/auton_control;STOP;{haveLedBlink}");
        currentTarget = null;

        if(script != null)
        {
            script.SetInactiveUI();
            StatusIndicator.instance.SetIndicator(Status.NotActivated, script);
            // this should be done on callback from the rover
        }
    }

    public void Stop2(){
        // for some reason I cant drag the original Stop script onto button in unity
        Stop();
    }

    void SendDriveForwards10Feet() {
        string message = $"autonomous/auton_control;DRIVEFORWARD";
        TcpController.inst.Publish(message);
    }

    IEnumerator WaitTwoSecondsThenSendCommand(string cmd) {
        yield return new WaitForSeconds(2f);
        TcpController.inst.Publish(cmd);
    }

    public void ManuallyFindItem(string item)
    {
        TcpController.inst.Publish($"autonomous/auton_control;FIND;{item}");
    }


    public void ReceivePositionUpdate(double lat, double lon) {
        if (currentTarget == null && squarePointIndex == -1) return;
        if (searchingForAruco) return;

        var worldPos = MapController.instance.GetWorldPosition(lat, lon);
        //var targetWorldPos = waypoints[waypointIndex];
        //Vector2 targetVect = MapController.instance.GetWorldPosition(targetWorldPos.lat, targetWorldPos.lon);
        var dist = Vector2.Distance(worldPos, currentVector2Target);
        Debug.Log("Distance to target: " + dist);

        if (dist < distanceCutoff) {
            if (isInReverse) waypointIndex--;
            else waypointIndex++;

            string nextCommand;
            if (squarePointIndex >= 0 && !searchingForAruco)
            {
                Debug.Log($"Reached search point. Target: {item}");
                SendDriveForwards10Feet();
                nextCommand = $"autonomous/auton_control;FIND;{item}";
                searchingForAruco = true;
                StartCoroutine(WaitTwoSecondsThenSendCommand(nextCommand));
            }
            else if (waypointIndex >= waypoints.Count)
            {
                Debug.Log($"Reached destination. Target: {item}");
                SendDriveForwards10Feet();
                if (item != ItemToFind.none)
                {
                    nextCommand = $"autonomous/auton_control;FIND;{item}";
                    searchingForAruco = true;
                }
                else
                {
                    Debug.Log("Set UI inactive");
                    currentTarget.SetInactiveUI();
                    StatusIndicator.instance.SetIndicator(Status.NotActivated, currentTarget);
                    bool haveLedBlink = true;
                    nextCommand = $"autonomous/auton_control;STOP;{haveLedBlink}";
                    currentTarget = null;
                }
                StartCoroutine(WaitTwoSecondsThenSendCommand(nextCommand));
            }
            else if (waypointIndex < 0)
            {
                Debug.Log("Returned");
                Stop(currentTarget, true);
            }
            else
            {
                Debug.Log("Send next waypoint");
                SendNextWaypoint();
            }
        }
    }

    public void ReceiveFeedback(string response) {

        var parts = response.Split(";");
        //Debug.Log(parts[1]);
        if(parts[1] == "scan failed") {
            squarePointIndex++;
            if (squarePointIndex > 11) {
                Stop(currentTarget);
                return;
            }

            var pos = squarePoints[squarePointIndex];
            var coords = MapController.instance.GetLatLonFromWorldPosition(pos);
            string message = $"autonomous/auton_control;GOTO;{coords[0]};{coords[1]};True";
            searchingForAruco = false;
            currentVector2Target = pos;
            MapController.instance.lineTarget = pos;
            TcpController.inst.Publish(message);
        }
        else if(parts[1] == "arrived"){
            Debug.Log("rover says it arrived. Current target: " + currentTarget);
            if(currentTarget != null)
            {
                currentTarget.SetInactiveUI();
                StatusIndicator.instance.SetIndicator(Status.NotActivated, currentTarget);
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
