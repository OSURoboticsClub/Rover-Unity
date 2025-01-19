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

    void OnImuReceived(string message)
    {
        // This method is called when the event is triggered
        Debug.Log($"Received message: {message}");
        var parts = message.Split(";");
        float x = float.Parse(parts[1]);
        float y = float.Parse(parts[2]);
        float z = float.Parse(parts[3]);
        float w = float.Parse(parts[4]);

        float yaw = Mathf.Atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
        yaw = yaw * Mathf.Rad2Deg; // Convert from radians to degrees
        roverIcon.rotation = Quaternion.Euler(0, 0, -yaw); // Negative yaw due to coordinate differences
    }

    void OnDestroy()
    {
        TcpMessageReceiver.gpsReceived.RemoveListener(OnGpsReceived);
        TcpMessageReceiver.gpsReceived.RemoveListener(OnImuReceived);
    }
}
