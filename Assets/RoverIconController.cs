using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverIconController : MonoBehaviour
{
    [SerializeField] Transform roverIcon;

    void Start()
    {
        TcpMessageReceiver.gpsReceived.AddListener(OnMessageReceived);
    }

    void OnMessageReceived(string message)
    {
        // This method is called when the event is triggered
        Debug.Log($"Received message: {message}");
    }

    void OnDestroy()
    {
        // Clean up: remove the listener to avoid memory leaks
        TcpMessageReceiver.gpsReceived.RemoveListener(OnMessageReceived);
    }
}
