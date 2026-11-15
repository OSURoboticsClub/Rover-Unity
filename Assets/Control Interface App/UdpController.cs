using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System.Text;
using Newtonsoft.Json.Linq;
using System;
using Cysharp.Threading.Tasks;
using System.Diagnostics;

public class UdpController : MonoBehaviour
{
    public static UdpController inst;
    private UdpClient udpSendClient;
    private UdpClient udpReceiveClient;
    private UdpClient configClient;

    private UdpClient srvClient;
    private UdpClient srvServer;

    private UdpClient actionSend;
    private UdpClient actionReceive;

    
    private IPEndPoint serverEndPoint;
    private IPEndPoint receiveEndPoint;

    private IPEndPoint configEndPoint;

    private IPEndPoint srvEndPoint;

    private IPEndPoint srvServerPoint;

    private IPEndPoint actionSendEndPoint;
    private IPEndPoint actionReceiveEndPoint;


    private bool isConnected;
    public bool disconnected;

    [Header("UDP Configuration")]
    public string serverIP = "127.0.0.1";
    private int UDPSendPort = 65434;
    private int UDPReceivePort = 65435;

    private int UDPConfigPort = 65436;

    private int UDPConfigClientReq = 65437;

    private int UDPServerPort = 65438;

    private int UDPActionSendPort = 65439;
    private int UDPActionRecievePort = 65440;

    

    [Header("Connection Status")]
    public bool showDebugLogs = true;

    // Store latest messages per topic
    private Dictionary<string, JObject> latestMessages = new Dictionary<string, JObject>();
    
    // Store latest service responses per service key
    private Dictionary<string, JObject> latestServiceResponses = new Dictionary<string, JObject>();

    // Store the latest feedback and reponses per action name
    private Dictionary<string, JObject> latestActionFeedback = new Dictionary<string, JObject>();

    public void Reconnect()
    {
        Start();
    }

    /**
     * Initializes the UDP clients used to communicate with ROS
     */
    void Start()
    {
        inst = this;

        try
        {
            // Main send client
            udpSendClient = new UdpClient();
            serverEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPSendPort);

            // Receive client
            udpReceiveClient = new UdpClient(UDPReceivePort);
            receiveEndPoint = new IPEndPoint(IPAddress.Any, 0);

            // Config client
            configClient = new UdpClient();
            configEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPConfigPort);

            srvClient = new UdpClient();
            srvEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPConfigClientReq);

            srvServer = new UdpClient(UDPServerPort);
            srvServerPoint = new IPEndPoint(IPAddress.Any, 0);

            // Actions
            actionSend = new UdpClient();
            actionSendEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), UDPActionSendPort);

            actionReceive = new UdpClient(UDPActionRecievePort);
            actionReceiveEndPoint = new IPEndPoint(IPAddress.Any, 0);

            isConnected = true;
            disconnected = false;

            if (showDebugLogs)
                UnityEngine.Debug.Log($"UDP Controller initialized: send:{serverIP}:{UDPSendPort}, receive:{UDPReceivePort}, client:{UDPConfigPort}");
        }
        catch (System.Exception e)
        {
            UnityEngine.Debug.LogError($"Could not initialize UDP Controller: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }

    /**
     * Checks for data sent from ROS and stores it in the appropriate dictionary
     */
    void Update()
    {
        // Continuously check for topic messages
        while (udpReceiveClient != null && udpReceiveClient.Available > 0)
        {
            try
            {
                byte[] data = udpReceiveClient.Receive(ref receiveEndPoint);
                string jsonString = Encoding.UTF8.GetString(data);
                JObject message = JObject.Parse(jsonString);

                // Update latest messages dictionary
                string topic = message["topic"]?.ToString();
                if (!string.IsNullOrEmpty(topic))
                {
                    latestMessages[topic] = message;
                    
                }
            }
            catch (Exception e)
            {
                UnityEngine.Debug.LogWarning($"Failed to receive or parse UDP message: {e.Message}");
            }
        }
        
        // Continuously check for service response messages
        while (srvServer != null && srvServer.Available > 0)
        {
            try
            {
                byte[] data = srvServer.Receive(ref srvServerPoint);
                string jsonString = Encoding.UTF8.GetString(data);
                JObject message = JObject.Parse(jsonString);

                // Update latest service responses dictionary
                string serviceKey = message["service"]?.ToString();
                UnityEngine.Debug.Log(message);
                UnityEngine.Debug.Log(serviceKey);
                if (!string.IsNullOrEmpty(serviceKey))
                {
                    latestServiceResponses[serviceKey] = message;
                    if (showDebugLogs)
                        UnityEngine.Debug.Log($"Updated service response for key: {serviceKey}");
                }
            }
            catch (Exception e)
            {
                UnityEngine.Debug.LogWarning($"Failed to receive or parse service UDP message: {e.Message}");
            }
        }

        // Continuously check for action feedback and/or response messages
        while(actionReceive != null && actionReceive.Available > 0)
        {
            try
            {
                byte[] data = actionReceive.Receive(ref actionReceiveEndPoint);
                string jsonString = Encoding.UTF8.GetString(data);
                JObject message = JObject.Parse(jsonString);

                // update latest action feedback dictionary
                string actionName = message["action"]?.ToString();
                UnityEngine.Debug.Log(message);
                UnityEngine.Debug.Log(actionName);
                if(!string.IsNullOrEmpty(actionName))
                {
                    latestActionFeedback[actionName] = message;
                    if(showDebugLogs)
                        UnityEngine.Debug.Log($"Updated action response for action: {actionName}");
                }
            }
            catch(Exception e)
            {
                UnityEngine.Debug.LogWarning($"Failed to receive or parse action UDP message: {e.Message}");
            }
        }
    }

    /**
     * Waits for a service to respond to a previously made request
     *
     * @param serviceKey the key for the service the request was made to
     * @param maxAttempts the maximum number of attempts made to try and get the response
     * @param delayBetweenAttempts how long (in seconds) to wait between each attempt
     * @return a JObject representing the response if recieved, or an empty JObject if the function times out
     */
    public async UniTask<JObject> WaitForServiceResponse(string serviceKey, int maxAttempts = 50, float delayBetweenAttempts = 0.1f)
    {
        for (int i = 0; i < maxAttempts; i++)
        {   
            foreach (var key in latestServiceResponses.Keys)
            {
                UnityEngine.Debug.Log("Key: " + key);
            }
            // Check if the response has been received
            if (latestServiceResponses.TryGetValue(serviceKey, out JObject response))
            {
                // Remove from dictionary after reading (optional, depending on your use case)
                latestServiceResponses.Remove(serviceKey);
                if (showDebugLogs)
                    UnityEngine.Debug.Log($"Service response found for key: {serviceKey}");
                
                return response;
            }

            await UniTask.Delay(TimeSpan.FromSeconds(delayBetweenAttempts));
        }

        UnityEngine.Debug.LogWarning($"WaitForServiceResponse timed out for key: {serviceKey}");
        return new JObject();
    }

    /**
     * Creates a subscription to a ROS topic
     *
     * @param topic the topic to subscribe to
     * @param msgType a ROS interface string describing the type of data that will be received
     */
    public void ConfigureSubscription(string topic, string msgType)
    {
        string message = topic + ";" + msgType;
        if (showDebugLogs)
            UnityEngine.Debug.Log($"Sending config UDP Message: {message}");

        if (!isConnected || configClient == null)
        {
            UnityEngine.Debug.LogWarning("UDP config connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || configClient == null)
            {
                UnityEngine.Debug.LogWarning("UDP config reconnect failed. Canceling publish.");
                return;
            }
        }

        try
        {
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            configClient.Send(dataToSend, dataToSend.Length, configEndPoint);

            disconnected = false;
        }
        catch (System.Exception e)
        {
            UnityEngine.Debug.LogWarning($"Error while sending UDP config data: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }

    /**
     * Publishes to a ROS topic
     *
     * @param message a string containing the JSON to be sent to ROS, in the format specified in README.md
     */
    public void PublishMessage(string message)
    {
        if (showDebugLogs)
            UnityEngine.Debug.Log($"Sending UDP Message: {message}");

        if (!isConnected || udpSendClient == null)
        {
            UnityEngine.Debug.LogWarning("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || udpSendClient == null)
            {
                UnityEngine.Debug.LogWarning("UDP reconnect failed. Canceling publish.");
                return;
            } 
        }

        try
        {
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            udpSendClient.Send(dataToSend, dataToSend.Length, serverEndPoint);

            disconnected = false;
        }
        catch (System.Exception e)
        {
            UnityEngine.Debug.LogWarning($"Error while sending UDP control data: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }

    /**
     * Makes a request to a ROS service and waits for a response
     *
     * @param message a string containing the JSON to be sent to ROS, in the format specified in README.md
     * @param serviceKey an optional parameter containing the key for the service the request is being made to
     * @return a JObject representing the response if recieved, or an empty JObject if the function times out
     */
    public async UniTask<JObject> PublishClientReq(string message, string serviceKey = null)
    {
        if (showDebugLogs)
            UnityEngine.Debug.Log($"Sending UDP Client Req: {message}");
        
        JObject SrvReturn = new JObject();

        if (!isConnected || srvClient == null)
        {
            UnityEngine.Debug.Log("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || srvClient == null)
            {
                UnityEngine.Debug.Log("UDP reconnect failed. Canceling publish.");
                return SrvReturn;
            }
        }

        try
        {
            // If no service key provided, try to extract from message or generate one
            if (string.IsNullOrEmpty(serviceKey))
            { 
                JObject msgObj = JObject.Parse(message);
                serviceKey = msgObj["service"].ToString();
            }
            
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            srvClient.Send(dataToSend, dataToSend.Length, srvEndPoint);

            disconnected = false;
            // Wait for the response using the new dictionary-based approach
            SrvReturn = await WaitForServiceResponse(serviceKey);

            return SrvReturn;
        }
        catch (System.Exception e)
        {
            UnityEngine.Debug.LogWarning($"Error while sending UDP client req: {e.Message}");
            disconnected = true;
            isConnected = false;
            return SrvReturn;
        }
    }

    /**
     * Requests ROS to start an action
     *
     * @param message a string containing the JSON to be sent to ROS, in the format specified in README.md
     * @param actionName an optional parameter containing the name for the action the request is being made to
     */
    public void PublishActionReq(string message, string actionName = null)
    {
        if (showDebugLogs)
            UnityEngine.Debug.Log($"Sending UDP Action Req: {message}");
        
        if (!isConnected || actionSend == null)
        {
            UnityEngine.Debug.Log("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || actionSend == null)
            {
                UnityEngine.Debug.Log("UDP reconnect failed. Canceling action request.");
                return;
            }
        }

        // If no service key provided, try to extract from message or generate one
        if (string.IsNullOrEmpty(actionName))
        {
            JObject msgObj = JObject.Parse(message);
            actionName = msgObj["action"].ToString();
        }
            
        byte[] dataToSend = Encoding.UTF8.GetBytes(message);
        actionSend.Send(dataToSend, dataToSend.Length, actionSendEndPoint);
    }

    /**
     * Makes a request to a ROS service and does not wait for a response
     *
     * @param message a string containing the JSON to be sent to ROS, in the format specified in README.md
     */
    public void SendClientReq(string message)
    {
        if (showDebugLogs)
            UnityEngine.Debug.Log($"Sending UDP Client Req (no wait): {message}");

        if (!isConnected || srvClient == null)
        {
            UnityEngine.Debug.Log("UDP connection is not established. Attempting reconnect.");
            Start();
            if (!isConnected || srvClient == null)
            {
                UnityEngine.Debug.Log("UDP reconnect failed. Canceling send.");
                return;
            }
        }

        try
        {
            byte[] dataToSend = Encoding.UTF8.GetBytes(message);
            srvClient.Send(dataToSend, dataToSend.Length, srvEndPoint);
            disconnected = false;
        }
        catch (System.Exception e)
        {
            UnityEngine.Debug.LogWarning($"Error while sending UDP client req: {e.Message}");
            disconnected = true;
            isConnected = false;
        }
    }


    /**
     * Gets the latest message for a ROS topic
     *
     * @param topic the ROS topic
     * @return a JObject representing the latest message if any have been recieved, or an empty JObject otherwise
     */
    public JObject GetLatestMessage(string topic)
    {
        latestMessages.TryGetValue(topic, out JObject msg);
        return msg;
    }
    
    /**
     * Gets the latest response from a service request
     *
     * @param serviceKey the key for the service
     * @return a JObject representing the latest response if any have been recieved, or an empty JObject otherwise
     */
    public JObject GetLatestServiceResponse(string serviceKey)
    {
        latestServiceResponses.TryGetValue(serviceKey, out JObject msg);
        return msg;
    }

    /**
     * Gets the latest feedback or results for a ROS action
     *
     * @param actionName the name of the action
     * @return a JObject representing the latest feedback or result if either have been recieved, or an empty JObject otherwise
     */
    public JObject GetLatestActionFeedback(string actionName)
    {
        latestActionFeedback.TryGetValue(actionName, out JObject msg);
        return msg;
    }

    void OnDestroy()
    {
        isConnected = false;

        if (udpSendClient != null)
        {
            try { udpSendClient.Close(); } catch (Exception e) { UnityEngine.Debug.LogWarning($"Error closing UDP client: {e.Message}"); }
            udpSendClient = null;
        }

        if (udpReceiveClient != null)
        {
            try { udpReceiveClient.Close(); } catch (Exception e) { UnityEngine.Debug.LogWarning($"Error closing UDP receive client: {e.Message}"); }
            udpReceiveClient = null;
        }
        
        if (srvClient != null)
        {
            try { srvClient.Close(); } catch (Exception e) { UnityEngine.Debug.LogWarning($"Error closing service client: {e.Message}"); }
            srvClient = null;
        }
        
        if (srvServer != null)
        {
            try { srvServer.Close(); } catch (Exception e) { UnityEngine.Debug.LogWarning($"Error closing service server: {e.Message}"); }
            srvServer = null;
        }

        if(actionSend != null)
        {
            try { actionSend.Close(); } catch (Exception e) { UnityEngine.Debug.LogWarning($"Error closing action sender: {e.Message}"); }
            actionSend = null;
        }

        if(actionReceive != null)
        {
            try { actionReceive.Close(); } catch (Exception e) { UnityEngine.Debug.LogWarning($"Error closing action receiver: {e.Message}"); }
            actionReceive = null;
        }

        if (showDebugLogs)
            UnityEngine.Debug.Log("UDP Control Controller closed.");
    }

    void OnApplicationPause(bool pauseStatus)
    {
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