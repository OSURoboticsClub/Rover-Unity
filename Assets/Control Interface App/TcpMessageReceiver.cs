using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using System.Collections.Concurrent;

public class TcpMessageReceiver : MonoBehaviour
{
    public static TcpMessageReceiver inst;
    private ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    [System.Serializable] public class GpsEvent : UnityEvent<string> { }
    public static GpsEvent gpsReceived = new GpsEvent();
    [System.Serializable] public class ImuEvent : UnityEvent<string> { }
    public static GpsEvent imuReceived = new GpsEvent();

    void Awake()
    {
        inst = this;   
    }

    public void Receive(string data){
        messageQueue.Enqueue(data);
    }

    void Update()
    {
        // Process messages on the main thread
        while (messageQueue.TryDequeue(out string message))
        {
            ProcessMessage(message);
        }
    }

    private void ProcessMessage(string message)
    {
        var parts = message.Split(";");
        if (parts[0] == "tower/status/gps"){
            gpsReceived.Invoke(message);
        }
        else if (parts[0] == "imu/data/heading")
        {
            imuReceived.Invoke(message);
        }
        else {
            MessagesController.inst.DisplayMessage(message);
        }

    }
}
