using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class ROSBridgeConnectionManager : MonoBehaviour
{
    [Header("Network Settings")]
    public string rosHost = "127.0.0.1";
    public int tcpPort = 5005;
    public int udpListenPort = 5006;

    private UdpClient udpClient;
    private Thread receiveThread;
    private bool isRunning = false;


    private MissionTelemetry latestTelemetry = new MissionTelemetry();
    private readonly object telemetryLock = new object();

    void Start()
    {
        latestTelemetry.timestamp = 0.0;
        isRunning = true;
        receiveThread = new Thread(ReceiveUDPData) { IsBackground = true };
        receiveThread.Start();
        Debug.Log($"RBCM started. Listening for UDP telemetry on {udpListenPort}");
    }

    [ContextMenu("Send Mission to ROS")]
    public void SendMission()
    {
        string missionJson = SatelliteMapSystem.Instance.GetMissionJson();
        if (string.IsNullOrEmpty(missionJson) || missionJson == "{}")
        {
            Debug.LogError("RBCM No mission to send.");
            return;
        }

        Debug.Log($"Sending Mission");
        try
        {
            Debug.Log($"Connecting to ROS node at {rosHost}:{tcpPort}");
            using (TcpClient client = new TcpClient(rosHost, tcpPort))
            {
                Debug.Log($"Connected to ROS node at {rosHost}:{tcpPort}");
                client.SendTimeout = 5000;
                using (NetworkStream stream = client.GetStream())
                {
                    Debug.Log($"Sending mission JSON: {missionJson}");
                    byte[] data = Encoding.UTF8.GetBytes(missionJson + "\n");
                    stream.Write(data, 0, data.Length);
                    stream.Flush();
                    Debug.Log("Mission sent to ROS node.");
                }
            }
        }
        catch (SocketException ex)
        {
            Debug.LogError($"RBCM Socket error: {ex}");
        }
    }

    private void ReceiveUDPData()
    {
        udpClient = new UdpClient(udpListenPort);
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, udpListenPort);

        while (isRunning)
        {
            try
            {
                byte[] data = udpClient.Receive(ref anyIP);
                string text = Encoding.UTF8.GetString(data);
                
                MissionTelemetry parsed = JsonUtility.FromJson<MissionTelemetry>(text);
                
                lock (telemetryLock)
                {
                    if (parsed.timestamp > latestTelemetry.timestamp)
                    {
                        latestTelemetry = parsed;
                    }
                }
            }
            catch (SocketException ex)
            {
                if (isRunning)
                {
                    Debug.LogWarning($"UDP Receive Exception: {ex.Message}");
                }
            }
        }
    }

    void Update()
    {
        MissionTelemetry currentData;
        
        lock (telemetryLock)
        {
            currentData = latestTelemetry;
        }

        // Validate data has been received before processing
        if (currentData.timestamp > 0)
        {
            SatelliteMapSystem.Instance.OnTelemetryReceived?.Invoke(currentData);
        }
    }

    void OnDestroy()
    {
        isRunning = false;
        if (udpClient != null)
        {
            udpClient.Close();
        }
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }
    }
}
