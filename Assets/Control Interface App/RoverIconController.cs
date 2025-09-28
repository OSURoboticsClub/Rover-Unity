using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverIconController : MonoBehaviour
{
    public static RoverIconController inst;
    public Transform roverIcon;

    [SerializeField] Vector3 angles;
    [SerializeField] float offset = -1000;
    [SerializeField] bool useOffset = true;
    [SerializeField] bool logHeading = true;
    [SerializeField] float buildingAngle = -20f;
    [SerializeField] GameObject circle;
    [SerializeField] public double latestlat = 0;
    [SerializeField] public double latestlon = 0;
    [SerializeField] Transform gpsCircleParent;

    public Vector3 latestGps;

    void Start()
    {
        inst = this;
        TcpMessageReceiver.gpsReceived.AddListener(OnGpsReceived);
        TcpMessageReceiver.imuReceived.AddListener(OnImuHeadingReceived);
        TcpMessageReceiver.simplePositionReceived.AddListener(OnSimplePositionReceived);
        //offset = -1000;
    }

    public float t = 3f;
    private void Update()
    {
        //Debug.Log(latestlat);
        //if (latestlat == 0) return;

        //t += Time.deltaTime;
        //if(t > 2f)
        //{
        //    t = 0;
        //    if (latestGps == Vector3.zero) return;
        //    var obj = Instantiate(circle, gpsCircleParent);
        //    obj.transform.position = latestGps;
        //    latestGps = Vector3.zero;
        //}

        // this was something I made to show dots on the map that represented the actual
        // gps data that was coming in
    }

    void OnGpsReceived(string message)
    {
        var parts = message.Split(";");
        latestlat = double.Parse(parts[1]);
        latestlon = double.Parse(parts[2]);
        latestGps = MapController.instance.GetWorldPosition(latestlon, latestlon);
        latestGps.y = 0;
        //Debug.Log($"GPS: {latestlat}, {latestlon}\nWorld pos: {latestGps}");
        //roverIcon.position = worldPos;
    }

    void OnImuHeadingReceived(string message)
    {
        //Debug.Log("Received IMU heading: " + message);
        var parts = message.Split(";");
        bool succ = float.TryParse(parts[1], out float result);
        result += offset;
        if(succ) roverIcon.rotation = Quaternion.Euler(0, 0, -result);
    }

    void OnSimplePositionReceived(string message)
    {
        var parts = message.Split(";");
        double lat = double.Parse(parts[1]);
        double lon = double.Parse(parts[2]);
        Vector2 worldPos = MapController.instance.GetWorldPosition(lat, lon);
        //Debug.Log($"SP: {lat}, {lon}\n" + "Set pos to " + worldPos);
        roverIcon.position = worldPos;
        CurrentDestinationController.inst.ReceivePositionUpdate(lat, lon);
    }

    void OnImuReceived(string message)
    {
        // This method is called when the event is triggered
        // Debug.Log($"Received message: {message}");
        var parts = message.Split(";");
        float x = float.Parse(parts[1]);
        float y = float.Parse(parts[2]);
        float z = float.Parse(parts[3]);
        float w = float.Parse(parts[4]);

        Quaternion quaternion = new Quaternion(x, y, z, w);

        float heading = GetHeading(quaternion);
        //if(offset == -1000){
        //    Debug.Log("Current heading: " + heading);
        //    float headingAdjusted = heading;
        //    if(heading > 180f) headingAdjusted -= 360f;
        //    float targetInitialHeading = 360f - buildingAngle;
        //    Debug.Log("Target initial heading: " + targetInitialHeading);
        //    offset = headingAdjusted - targetInitialHeading;
        //    if(offset < -180) offset += 360;
        //    offset *= -1;

        //    Debug.Log("Offset is now: " + offset);
        //}

        if(useOffset) heading += offset;
        if(heading < 0) heading += 360f;
        if(logHeading) Debug.Log(heading);
        
        roverIcon.rotation = Quaternion.Euler(0, 0, heading); // Negative yaw due to coordinate differences
    }

    // Helper function to normalize angles
    float GetHeading(Quaternion q)
    {
        // roll (φ) around X axis
        float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        float roll = Mathf.Atan2(sinr_cosp, cosr_cosp) * Mathf.Rad2Deg;
        
        // Optional: keep in 0–360 if you prefer
        //if (roll < 0) roll += 360f; 
        return -roll + 180;
    }

    void OnDestroy()
    {
        TcpMessageReceiver.gpsReceived.RemoveListener(OnGpsReceived);
        TcpMessageReceiver.imuReceived.RemoveListener(OnImuHeadingReceived);
        TcpMessageReceiver.simplePositionReceived.RemoveListener(OnSimplePositionReceived);
    }
}
