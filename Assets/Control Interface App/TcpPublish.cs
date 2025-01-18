using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class TcpPublish : MonoBehaviour
{
    public static TcpPublish inst;
    private TcpClient client;
    private NetworkStream stream;
    private Thread listenThread;
    private bool isRunning;

    // Start is called before the first frame update
    void Start()
    {
        inst = this;
        string server = "127.0.0.1"; // Server IP
        int port = 65432;
        try
        {
            client = new TcpClient(server, port);
            stream = client.GetStream();
            isRunning = true;

            Debug.Log($"Connected to server at {server}:{port}");

            // Start a thread to listen for incoming data
            listenThread = new Thread(ListenForData);
            listenThread.IsBackground = true;
            listenThread.Start();
        }
        catch (SocketException e)
        {
            Debug.LogError($"Could not connect to server: {e.Message}");
        }
    }

    public void Publish(string cmd, double latitude, double longitude)
    {
        if (client == null || stream == null)
        {
            Debug.LogError("TCP connection is not established. Unable to send data.");
            return;
        }

        try
        {
            // Format the message
            string message = $"{cmd};{latitude};{longitude}";
            byte[] dataToSend = Encoding.ASCII.GetBytes(message);

            // Send the message
            stream.Write(dataToSend, 0, dataToSend.Length);
            Debug.Log($"Sent: {message}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error while sending data: {e.Message}");
        }
    }

    private void ListenForData()
    {
        try
        {
            byte[] buffer = new byte[1024];
            while (isRunning && client != null && stream != null)
            {
                // Check if data is available
                if (stream.DataAvailable)
                {
                    int bytesRead = stream.Read(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        string receivedMessage = Encoding.ASCII.GetString(buffer, 0, bytesRead);
                        // Trigger a callback or handle the received data
                        OnDataReceived(receivedMessage);
                    }
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error while listening for data: {e.Message}");
        }
    }

    private void OnDataReceived(string data)
    {
        // Handle the data received from the server
        TcpMessageReceiver.inst.Receive(data);
    }

    void OnDestroy()
    {
        // Cleanup
        isRunning = false;

        if (listenThread != null && listenThread.IsAlive)
        {
            listenThread.Join();
        }

        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();

        Debug.Log("TCP connection closed.");
    }
}
