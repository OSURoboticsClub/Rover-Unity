using UnityEngine;
using System.Diagnostics;
using System.Collections.Generic;
using TMPro;

public class GStreamerLauncher : MonoBehaviour
{
    public string portNum;
    public string sourcePort;
    private Process gStreamerProcess;
    public TMP_Dropdown dropdown;

    private Dictionary<int, string> cameraPortMap = new Dictionary<int, string>()
    {
        { 0, "42067" },
        { 1, "42068" },
        { 2, "42069" },
        { 3, "42074" },
        { 4, "42075" },
    };

    public int defaultCameraIndex = 3;

    void Start()
    {
        // Select the default camera port at startup, but keep launching manual through the UI.
        if (dropdown != null)
        {
            dropdown.value = defaultCameraIndex;
            dropdown.RefreshShownValue();
            dropdown.onValueChanged.AddListener(OnDropdownChanged);
        }

        if (string.IsNullOrWhiteSpace(sourcePort) && cameraPortMap.ContainsKey(defaultCameraIndex))
        {
            sourcePort = cameraPortMap[defaultCameraIndex];
        }
    }

    public void OnDropdownChanged(int index)
    {
        if (cameraPortMap.ContainsKey(index))
        {
            StopGStreamer();
            sourcePort = cameraPortMap[index];
            LaunchGStreamer();
        }
        else
        {
            UnityEngine.Debug.LogWarning("No port mapping found for dropdown index: " + index);
        }
    }

    public void LaunchGStreamer()
    {
        // If the last gst-launch process already exited, clear it so the button can restart it.
        if (gStreamerProcess != null && gStreamerProcess.HasExited)
        {
            gStreamerProcess.Dispose();
            gStreamerProcess = null;
        }

        if (gStreamerProcess != null)
        {
            return;
        }

        if (string.IsNullOrWhiteSpace(sourcePort) || string.IsNullOrWhiteSpace(portNum))
        {
            UnityEngine.Debug.LogWarning("Cannot launch GStreamer without both sourcePort and portNum.");
            return;
        }

        ProcessStartInfo startInfo = new ProcessStartInfo();
        startInfo.FileName = "/bin/bash";
        // Decode the incoming UDP H265 stream and forward raw RGB frames to CameraListener over TCP.
        startInfo.Arguments = "-c \"gst-launch-1.0 udpsrc port=" + sourcePort +
            " caps=\\\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265\\\" " +
            "! rtpulpfecdec ! rtpjitterbuffer latency=200 ! rtph265depay ! h265parse ! " +
            "queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 ! " +
            "avdec_h265 ! videoconvert ! videorate ! video/x-raw,format=RGB,framerate=30/1 ! " +
            "tcpclientsink host=127.0.0.1 port=" + portNum + " sync=false\"";
        startInfo.UseShellExecute = false;
        startInfo.RedirectStandardOutput = true;
        startInfo.RedirectStandardError = true;

        gStreamerProcess = new Process();
        gStreamerProcess.StartInfo = startInfo;
        gStreamerProcess.OutputDataReceived += (sender, args) =>
        {
            if (!string.IsNullOrEmpty(args.Data)) UnityEngine.Debug.Log(args.Data);
        };
        gStreamerProcess.ErrorDataReceived += (sender, args) =>
        {
            if (!string.IsNullOrEmpty(args.Data)) UnityEngine.Debug.Log(args.Data);
        };
        gStreamerProcess.Start();
        gStreamerProcess.BeginOutputReadLine();
        gStreamerProcess.BeginErrorReadLine();

        UnityEngine.Debug.Log($"GStreamer launched from UDP {sourcePort} to TCP {portNum}");
    }
 
    public void StopGStreamer()
    {
        // Stop the manually launched GStreamer process without changing the selected camera.
        if (gStreamerProcess != null)
        {
            if (!gStreamerProcess.HasExited)
            {
                gStreamerProcess.Kill();
                gStreamerProcess.WaitForExit();
            }

            gStreamerProcess.Dispose();
            gStreamerProcess = null;
        }
    }

    void OnApplicationQuit()
    {
        StopGStreamer();
    }
}
