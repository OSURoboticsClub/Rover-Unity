using System.Net;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.UI;

public class MJPEGUdpReceiverNoThread : MonoBehaviour
{
    public int listenPort = 5000;
    public int timeoutCount = 60;
    public Texture timeoutTexture;
    public RawImage cameraImage;
    
    
    private int count = 0;
    

    private UdpClient udpClient;
    private IPEndPoint remoteEP;

    private Texture2D receivedTexture;

    void Start()
    {
        if (cameraImage == null)
        {
            Debug.LogError("Please assign a RawImage UI element to display the video.");
            enabled = false;
            return;
        }

        udpClient = new UdpClient(listenPort);
        udpClient.Client.ReceiveTimeout = 1; // 1 ms timeout to avoid blocking

        remoteEP = new IPEndPoint(IPAddress.Any, listenPort);
        receivedTexture = new Texture2D(2, 2, TextureFormat.RGBA32, false);
    }

    void Update()
    {
        try
        {
            if (udpClient.Available > 0)
            {
                count = 0;
                byte[] data = udpClient.Receive(ref remoteEP);
                if (data != null && data.Length > 0)
                {
                    bool loaded = receivedTexture.LoadImage(data);
                    if (loaded)
                    {
                        cameraImage.texture = receivedTexture;

                        // Optional: adjust aspect ratio to fit
                        float aspectRatio = (float)receivedTexture.width / receivedTexture.height;
                        RectTransform rt = cameraImage.GetComponent<RectTransform>();
                        rt.sizeDelta = new Vector2(rt.sizeDelta.y * aspectRatio, rt.sizeDelta.y);
                    }
                    else
                    {
                        Debug.LogWarning("Failed to load JPEG image.");
                    }
                }
            }
            else {
            	count += 1;
            	if (count >= timeoutCount){
            	    if (cameraImage.texture != timeoutTexture) {
            	        cameraImage.texture = timeoutTexture;
            	    }
            	    count = timeoutCount;
            	}
            }
        }
        catch (SocketException)
        {
            
        }
    }

    void OnApplicationQuit()
    {
        if (udpClient != null)
        {
            udpClient.Close();
        }
    }
}

