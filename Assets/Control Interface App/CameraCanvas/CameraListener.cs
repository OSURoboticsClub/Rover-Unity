using System.Net;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.UI;
using System;

public class RGBTcpReceiver : MonoBehaviour
{
    public int listenPort = 5000;
    public int frameWidth = 640;
    public int frameHeight = 480;
    public int timeoutCount = 120;
    public Texture timeoutTexture;
    public RawImage cameraImage;

    private int count = 0;
    private TcpListener tcpListener;
    private TcpClient tcpClient;
    private NetworkStream stream;
    private Texture2D receivedTexture;
    private byte[] frameBuffer;
    private int frameSize;
    private int bytesReceived = 0;
    private bool isListening = false;

    void Start()
    {
        if (cameraImage == null)
        {
            Debug.LogError("Please assign a RawImage UI element to display the video.");
            enabled = false;
            return;
        }

        // RGB = 3 bytes per pixel
        frameSize = frameWidth * frameHeight * 3;
        frameBuffer = new byte[frameSize];
        receivedTexture = new Texture2D(frameWidth, frameHeight, TextureFormat.RGB24, false);

        StartListening();
    }

    void Update()
    {
        try
        {
            if (!isListening)
            {
                // Restart the listener if Unity disabled it or a socket error stopped it.
                StartListening();
                return;
            }

            // Accept new connections if we don't have one.
            if (tcpClient == null || !tcpClient.Connected)
            {
                if (tcpListener.Pending())
                {
                    tcpClient = tcpListener.AcceptTcpClient();
                    stream = tcpClient.GetStream();
                    // 1 ms timeout keeps Unity's main thread from hanging on reads.
                    stream.ReadTimeout = 1;
                    bytesReceived = 0;
                    Debug.Log($"Camera TCP client connected on port {listenPort}");
                }
                else
                {
                    return;
                }
            }

            if (stream != null && stream.DataAvailable)
            {
                count = 0;

                // Read as much data as is available for the current RGB frame.
                int bytesToRead = frameSize - bytesReceived;
                int read = stream.Read(frameBuffer, bytesReceived, bytesToRead);
                bytesReceived += read;

                // If we have a complete frame, load the raw RGB data into the texture.
                if (bytesReceived >= frameSize)
                {
                    receivedTexture.LoadRawTextureData(frameBuffer);
                    receivedTexture.Apply();
                    cameraImage.texture = receivedTexture;
                    // Reset for next frame.
                    bytesReceived = 0;
                }
            }
            else if (tcpClient != null && !tcpClient.Connected)
            {
                // Connection lost; keep the listener alive for the next GStreamer process.
                Debug.Log("Camera TCP client disconnected");
                CloseClient();
            }

            count++;
            if (count >= timeoutCount)
            {
                if (cameraImage.texture != timeoutTexture)
                {
                    cameraImage.texture = timeoutTexture;
                }

                count = 0;
                CloseClient();
            }
        }
        catch (System.IO.IOException)
        {
            // Timeout or short read; keep the connection alive.
        }
        catch (ObjectDisposedException)
        {
            CloseClient();
        }
        catch (InvalidOperationException e)
        {
            Debug.LogWarning($"TCP listener on port {listenPort} was not active: {e.Message}");
            StopListening();
        }
        catch (SocketException e)
        {
            Debug.LogWarning($"Socket exception on camera port {listenPort}: {e.Message}");
            CloseClient();
        }
    }

    void StartListening()
    {
        if (isListening)
        {
            return;
        }

        try
        {
            tcpListener = new TcpListener(IPAddress.Any, listenPort);
            tcpListener.Start();
            isListening = true;
            Debug.Log($"TCP listener started on port {listenPort}");
        }
        catch (SocketException e)
        {
            isListening = false;
            tcpListener = null;
            Debug.LogWarning($"Could not start TCP listener on port {listenPort}: {e.Message}");
        }
    }

    void CloseClient()
    {
        // Close just the active TCP client; the listener remains available.
        stream?.Close();
        tcpClient?.Close();
        stream = null;
        tcpClient = null;
        bytesReceived = 0;
    }

    void StopListening()
    {
        // Full shutdown for Unity lifecycle events.
        CloseClient();

        if (tcpListener != null)
        {
            try { tcpListener.Stop(); } catch (Exception) { }
            tcpListener = null;
        }

        isListening = false;
    }

    void OnDisable()
    {
        StopListening();
    }

    void OnApplicationQuit()
    {
        StopListening();
    }
}
