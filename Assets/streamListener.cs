using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UI;

public class streamListener : MonoBehaviour
{
    //[SerializeField] int currentFrameNum = 0;
    [SerializeField] Image image;
    [SerializeField] float fps;
    float fpsCounter = 0;
    [SerializeField] float timeCounter = 0;
    int framesReceived = 0;

    [SerializeField] int frame1Num = 0;
    [SerializeField] int frame2Num = 0;
    [SerializeField] int frame3Num = 0;
    [SerializeField] List<float> lastReceivedTimes = new List<float>() { 0, 0, 0 };

    private Dictionary<int, byte[]> receivedPacketsFrame1 = new Dictionary<int, byte[]>();
    private Dictionary<int, byte[]> receivedPacketsFrame2 = new Dictionary<int, byte[]>();
    private Dictionary<int, byte[]> receivedPacketsFrame3 = new Dictionary<int, byte[]>();

    [SerializeField] string buffer1Status;
    [SerializeField] string buffer2Status;
    [SerializeField] string buffer3Status;

    bool frame1Done = false;
    bool frame2Done = false;
    bool frame3Done = false;

    public int listenPort = 12345;
    private UdpClient udpClient;

    private void Start()
    {
        udpClient = new UdpClient(listenPort);
        udpClient.Client.ReceiveBufferSize = 600000;
        Task.Factory.StartNew(() => ListenForData(), TaskCreationOptions.LongRunning);
    }

    private async void ListenForData()
    {
        while (true)
        {
            try
            {
                UdpReceiveResult result = await udpClient.ReceiveAsync();
                ProcessPacket(result.Buffer);
            }
            catch
            {
                //Debug.LogError($"UDP Receive Error: {e.Message}");
                break;
            }
        }
    }

    private void ProcessPacket(byte[] receivedData)
    {
        if (receivedData.Length < 16) return;  // Prevent out-of-bounds error

        int feedID = BitConverter.ToInt32(receivedData, 0);
        int frameNumber = BitConverter.ToInt32(receivedData, 4);
        int packetIndex = BitConverter.ToInt32(receivedData, 8);
        int numOfPackets = BitConverter.ToInt32(receivedData, 12);

        if(frame1Num > -1 && frame2Num > -1 && frame3Num > -1)
        {
            if (frameNumber != frame1Num && frameNumber != frame2Num && frameNumber != frame3Num)
                return;
        }

        byte[] imageDataPart = new byte[receivedData.Length - 16];

        Buffer.BlockCopy(receivedData, 16, imageDataPart, 0, receivedData.Length - 16);

        Receive(imageDataPart, frameNumber, packetIndex, numOfPackets);
    }

    private void OnApplicationQuit()
    {
        udpClient?.Close();
    }

    public void Receive(byte[] data, int frameNum, int packetIndex, int numOfPackets)
    {
        if(frameNum == frame1Num)
        {
            AddToFrameDictionary(receivedPacketsFrame1, numOfPackets, packetIndex, data, 0);
        }
        else if(frameNum == frame2Num)
        {
            AddToFrameDictionary(receivedPacketsFrame2, numOfPackets, packetIndex, data, 1);
        }
        else if (frameNum == frame3Num)
        {
            AddToFrameDictionary(receivedPacketsFrame3, numOfPackets, packetIndex, data, 2);
        }
        else
        {
            if(frame1Num == -1)
            {
                frame1Num = frameNum;
                AddToFrameDictionary(receivedPacketsFrame1, numOfPackets, packetIndex, data, 0);
            }
            else if (frame2Num == -1)
            {
                frame2Num = frameNum;
                AddToFrameDictionary(receivedPacketsFrame2, numOfPackets, packetIndex, data, 1);
            }
            else if (frame3Num == -1)
            {
                frame3Num = frameNum;
                AddToFrameDictionary(receivedPacketsFrame3, numOfPackets, packetIndex, data, 2);
            }
        }
    }

    void AddToFrameDictionary(Dictionary<int, byte[]> dict, int numOfPackets, int index, byte[] data, int bufferNum)
    {
        dict[index] = data;
        lastReceivedTimes[bufferNum] = timeCounter;
        string msg = "Rec. " + index + " / " + numOfPackets;
        if (bufferNum == 0) buffer1Status = msg;
        if (bufferNum == 1) buffer2Status = msg;
        if (bufferNum == 2) buffer3Status = msg;

        if (dict.Count == numOfPackets)
        {
            //Debug.Log("Completed frame " + frame1Num);

            framesReceived++;
            if (bufferNum == 0) frame1Done = true;
            if (bufferNum == 1) frame2Done = true;
            if (bufferNum == 2) frame3Done = true;
        }
    }

    private void Update()
    {
        timeCounter += Time.deltaTime;
        fpsCounter += Time.deltaTime;

        if(fpsCounter >= 2f)
        {
            fps = framesReceived / 2f;
            fpsCounter = 0;
            framesReceived = 0;
        }

        if (frame1Done)
        {
            var tex = ReconstructImage(receivedPacketsFrame1);
            Sprite newSprite = Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));
            image.sprite = newSprite;
            frame1Num = -1;
            receivedPacketsFrame1.Clear();
        }
        else if (frame2Done)
        {
            var tex = ReconstructImage(receivedPacketsFrame2);
            Sprite newSprite = Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));
            image.sprite = newSprite;
            frame2Num = -1;
            receivedPacketsFrame2.Clear();
        }
        else if (frame3Done)
        {
            var tex = ReconstructImage(receivedPacketsFrame3);
            Sprite newSprite = Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));
            image.sprite = newSprite;
            frame3Num = -1;
            receivedPacketsFrame3.Clear();
        }

        frame1Done = false;
        frame2Done = false;
        frame3Done = false;

        if(timeCounter - lastReceivedTimes[0] > .15f)
        {
            frame1Num = -1;
            receivedPacketsFrame1.Clear();
            lastReceivedTimes[0] = 0;
            buffer1Status = "";
        }
        if (timeCounter - lastReceivedTimes[1] > .15f)
        {
            frame2Num = -1;
            receivedPacketsFrame2.Clear();
            lastReceivedTimes[1] = 0;
            buffer2Status = "";
        }
        if (timeCounter - lastReceivedTimes[2] > .15f)
        {
            frame3Num = -1;
            receivedPacketsFrame3.Clear();
            lastReceivedTimes[2] = 0;
            buffer3Status = "";
        }
    }

    Texture2D ReconstructImage(Dictionary<int, byte[]> packets)
    {
        var orderedPackets = packets.OrderBy(kv => kv.Key); // Ensure correct order

        using (MemoryStream ms = new MemoryStream())
        {
            foreach (var pair in packets)
            {
                byte[] packetData = packets[pair.Key];
                ms.Write(packetData, 0, packetData.Length);
            }

            byte[] finalImageData = ms.ToArray();
            Texture2D texture = new Texture2D(2, 2);
            texture.LoadImage(finalImageData);
            texture.Apply();
            return texture;
        }
    }
}
