using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Threading;

public class UdpController : MonoBehaviour
{
    // Handles low-latency control communication over UDP port 65434 with the udp_control_relay node
    // This is for control commands only (drive, pan/tilt, joint angles)
    // Status updates should continue using TcpController
    
    public static UdpController inst;
    private UdpClient udpClient;
    private IPEndPoint serverEndPoint;
    private bool isConnected;
    public bool disconnected;
    
    [Header("UDP Configuration")]
    public string serverIP = "127.0.0.1";
    public int controlPort = 65434;
    
    [Header("Connection Status")]
    public bool showDebugLogs = true;
    
    public void Reconnect()
    {
        Start();
    }
    
    void Start()
    {
        inst = this;
        
        try
        {
            // Create UDP client
            udpClient = new UdpClient();
            serverEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), controlPort);
            
            isConnected = true;
            disconnected = false;
            
            if (showDebugLogs)
                Debug.Log($"UDP Control Controller initialized for {serverIP}:{controlPort}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Could not initialize UDP Control Controller: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }
    
    public void PublishControl(string message)
    {
        if (showDebugLogs)
            Debug.Log($"Sending UDP Control: {message}");
            
        if (!isConnected || udpClient == null)
        {
            Debug.LogWarning("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || udpClient == null)
            {
                Debug.LogWarning("UDP reconnect failed. Canceling publish.");
                return;
            }
        }
        
        try
        {
            // Convert message to bytes
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            
            // Send the message via UDP
            udpClient.Send(dataToSend, dataToSend.Length, serverEndPoint);
            
            disconnected = false;
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error while sending UDP control data: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }

    public void SendDriveCommand(bool controllerPresent, bool ignoreDriveControl, float linearX, float angularZ)
    {
        string message = $"command_control/ground_station_drive;{controllerPresent};{ignoreDriveControl};{linearX:F3};{angularZ:F3}";
        PublishControl(message);
    }
    
   
    public void SendPanTiltCommand(string topicName, bool shouldCenter, int panAdjustment, int tiltAdjustment, bool hitchServoPositive = false, bool hitchServoNegative = false)
    {
        string message = $"{topicName};{shouldCenter};{panAdjustment};{tiltAdjustment};{hitchServoPositive};{hitchServoNegative}";
        PublishControl(message);
    }
    
  
    public void SendChassisPanTilt(bool shouldCenter, int panAdjustment, int tiltAdjustment, bool hitchServoPositive = false, bool hitchServoNegative = false)
    {
        SendPanTiltCommand("chassis/pan_tilt/control", shouldCenter, panAdjustment, tiltAdjustment, hitchServoPositive, hitchServoNegative);
    }

    public void SendTowerPanTilt(bool shouldCenter, int panAdjustment, int tiltAdjustment, bool hitchServoPositive = false, bool hitchServoNegative = false)
    {
        SendPanTiltCommand("tower/pan_tilt/control", shouldCenter, panAdjustment, tiltAdjustment, hitchServoPositive, hitchServoNegative);
    }
    

    public void SendJointAngles(float[] angles)
    {
        if (angles.Length != 6)
        {
            Debug.LogError("Joint angles array must contain exactly 6 values");
            return;
        }
        
        string message = $"set_joint_angles;{angles[0]:F3};{angles[1]:F3};{angles[2]:F3};{angles[3]:F3};{angles[4]:F3};{angles[5]:F3}";
        PublishControl(message);
    }
    
    public void SendJoy2Command()
    {
        string message = "joy2;";
        PublishControl(message);
    }
    
    public void SendControlCommand(string topic, params string[] parameters)
    {
        string message = topic + ";" + string.Join(";", parameters);
        PublishControl(message);
    }
    
    void OnDestroy()
    {
        // Cleanup
        isConnected = false;
        
        if (udpClient != null)
        {
            try
            {
                udpClient.Close();
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"Error closing UDP client: {e.Message}");
            }
            udpClient = null;
        }
        
        if (showDebugLogs)
            Debug.Log("UDP Control Controller closed.");
    }
    
    void OnApplicationPause(bool pauseStatus)
    {
        // Handle application pause/resume
        if (pauseStatus)
        {
            isConnected = false;
        }
        else if (!isConnected)
        {
            Start();
        }
    }
}