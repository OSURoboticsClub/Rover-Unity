using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverIconController : MonoBehaviour
{
    [SerializeField] Transform roverIcon;

    void Start()
    {
        TcpMessageReceiver.gpsReceived.AddListener(OnGpsReceived);
        TcpMessageReceiver.imuReceived.AddListener(OnImuReceived);
    }

    void OnGpsReceived(string message)
    {
        Debug.Log($"Received message: {message}");
        var parts = message.Split(";");
        double lat = double.Parse(parts[1]);
        double lon = double.Parse(parts[2]);
        Vector2 worldPos = MapController.instance.GetWorldPosition(lat, lon);
        roverIcon.position = worldPos;
    }

    [SerializeField] Vector3 angles;
    [SerializeField] float offset;
    void OnImuReceived(string message)
    {
        // This method is called when the event is triggered
        Debug.Log($"Received message: {message}");
        var parts = message.Split(";");
        float x = float.Parse(parts[1]);
        float y = float.Parse(parts[2]);
        float z = float.Parse(parts[3]);
        float w = float.Parse(parts[4]);

        Quaternion quaternion = new Quaternion(x, y, z, w);
        angles = quaternion.eulerAngles;

        // float roll = Mathf.Atan2(
        //     2.0f * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
        //     1.0f - 2.0f * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        // );
        //yaw = yaw * Mathf.Rad2Deg; // Convert from radians to degrees
        roverIcon.rotation = Quaternion.Euler(0, 0, angles.x + offset); // Negative yaw due to coordinate differences
    }

    void OnDestroy()
    {
        TcpMessageReceiver.gpsReceived.RemoveListener(OnGpsReceived);
        TcpMessageReceiver.gpsReceived.RemoveListener(OnImuReceived);
    }
}
