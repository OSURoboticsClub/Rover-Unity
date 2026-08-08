using UnityEngine;
using System.Diagnostics;
using System.Collections.Generic;
using TMPro;
using Newtonsoft.Json.Linq;

using Debug = UnityEngine.Debug;

public class GStreamerLauncher : MonoBehaviour
{
    public string portNum;
    private string sourcePort;
    private string sourceFramerate;
    private Process gStreamerProcess;
    public TMP_Dropdown dropdown;

    // std_srvs/srv/SetBool template (Assets/Templates/std_srvs/srv/SetBool.json).
    // Assign in the inspector; used to build toggle_camera requests for the UDP bridge.
    [SerializeField] private TextAsset setBoolServiceJson;

    private string mulitcastAddr = "239.0.0.1";
    private string tcpAddr = "192.168.1.11";

    private class CameraEntry
    {
        public string name;
        public int port;
        public string framerate;
        public string serviceNamespace;

        public CameraEntry(string name, int port, string framerate, string serviceNamespace)
        {
            this.name = name;
            this.port = port;
            this.framerate = framerate;
            this.serviceNamespace = serviceNamespace;
        }
    }

    private CameraEntry activeEntry;

    // ROS service name the UDP bridge forwards each camera toggle to.
    private static string ToggleServiceName(CameraEntry entry) => $"/{entry.serviceNamespace}/toggle_camera";

    private void ToggleCameraThenLaunch(CameraEntry entry)
    {
        // Toggle the previously active camera off (fire-and-forget), if switching.
        if (activeEntry != null && activeEntry.serviceNamespace != entry.serviceNamespace)
        {
            SetCameraState(activeEntry, false);
        }

        // Toggle the new camera on, then launch without waiting for a response.
        SetCameraState(entry, true);
        activeEntry = entry;
        StartGStreamerProcess();
    }

    // Resolve the camera the UI currently has selected (dropdown wins, else the default).
    private CameraEntry GetSelectedEntry()
    {
        int index = (dropdown != null) ? dropdown.value : defaultCameraIndex;
        if (cameraPortMap.TryGetValue(index, out CameraEntry entry))
        {
            return entry;
        }
        return null;
    }

    // Send a toggle_camera SetBool request over the UDP bridge. Fire-and-forget: we do
    // not wait for or decode the response.
    private void SetCameraState(CameraEntry entry, bool state)
    {
        if (setBoolServiceJson == null)
        {
            Debug.LogError("[GStreamerLauncher] setBoolServiceJson TextAsset is not assigned in the inspector.");
            return;
        }

        if (UdpController.inst == null)
        {
            Debug.LogError("[GStreamerLauncher] UdpController.inst is null - no UDP controller in the scene.");
            return;
        }

        JObject msg = JObject.Parse(setBoolServiceJson.text);
        msg["service"] = ToggleServiceName(entry);
        msg["request"]["data"] = state;

        Debug.Log($"[{entry.serviceNamespace}] toggle_camera({state}) sent (no wait).");
        UdpController.inst.SendClientReq(msg.ToString());
    }

    private Dictionary<int, CameraEntry> cameraPortMap = new Dictionary<int, CameraEntry>()
    {
        {0, new CameraEntry("IR",            42073, "30/1", "chassis_gimbal")},
        {1, new CameraEntry("Tower",         42068, "30/1", "tower_gimbal")},
        {2, new CameraEntry("Chassis Right", 42070, "25/1", "chassis_right")},
        {3, new CameraEntry("Chassis Left",  42069, "25/1", "chassis_left")},
        {4, new CameraEntry("Gripper",       42074, "30/1", "d405")},
        {5, new CameraEntry("Back",          42071, "25/1", "back")},
        {6, new CameraEntry("BEV",           42072, "25/1", "bev")},
        {7, new CameraEntry("Chassis",       42075, "30/1", "d455")} //Not Used?
    };

    public int defaultCameraIndex = 3;

    void Start()
    {
        // Camera toggles now go through the custom UDP ROS bridge (UdpController) rather
        // than a per-camera ROS2 client, so there is no client setup to do here.

        // Select the default camera port at startup, but keep launching manual through the UI.
        if (dropdown != null)
        {
            dropdown.value = defaultCameraIndex;
            dropdown.RefreshShownValue();
            dropdown.onValueChanged.AddListener(OnDropdownChanged);
        }

        if (string.IsNullOrWhiteSpace(sourcePort) && cameraPortMap.ContainsKey(defaultCameraIndex))
        {
            sourcePort = cameraPortMap[defaultCameraIndex].port.ToString();
            sourceFramerate = cameraPortMap[defaultCameraIndex].framerate;
        }
    }

    public void OnDropdownChanged(int index)
    {
        Debug.Log($"[OnDropdownChanged] index={index}, GameObject={gameObject.name}, instanceID={GetInstanceID()}");
        if (cameraPortMap.ContainsKey(index))
        {
            StopGStreamer();
            CameraEntry entry = cameraPortMap[index];
            sourcePort = entry.port.ToString();
            sourceFramerate = entry.framerate;
            Debug.Log($"[OnDropdownChanged] set sourcePort={sourcePort}, sourceFramerate={sourceFramerate} on instanceID={GetInstanceID()}");
            ToggleCameraThenLaunch(entry);
        }
        else
        {
            Debug.LogWarning("No port mapping found for dropdown index: " + index);
        }
    }

    // Public entrypoint (e.g. a Launch button): toggle the selected camera's service on, then launch.
    public void LaunchGStreamer()
    {
        CameraEntry entry = GetSelectedEntry();
        if (entry == null)
        {
            Debug.LogWarning("[LaunchGStreamer] No camera selected - cannot toggle service or launch.");
            return;
        }

        sourcePort = entry.port.ToString();
        sourceFramerate = entry.framerate;
        ToggleCameraThenLaunch(entry);
    }

    private void StartGStreamerProcess()
    {
        Debug.Log($"[LaunchGStreamer] called on GameObject={gameObject.name}, instanceID={GetInstanceID()}, sourcePort='{sourcePort}', portNum='{portNum}'");
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
        /*startInfo.Arguments = "-c \"gst-launch-1.0 tcpclientsrc port=" + sourcePort + " host=" + tcpAddr +
            " ! \\\"application/x-rtp-stream, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265\\\" " +
            "! rtpstreamdepay ! rtpulpfecdec ! rtpjitterbuffer latency=200 ! rtph265depay ! h265parse ! " +
            "queue leaky=downstream max-size-buffers=10 max-size-time=0 max-size-bytes=0 ! " +
            "avdec_h265 ! videoconvert ! videorate ! video/x-raw,format=RGB,framerate=" + sourceFramerate + " ! " +
            "tcpclientsink host=127.0.0.1 port=" + portNum + " sync=false\"";
        */
        startInfo.Arguments = "-c \"gst-launch-1.0 udpsrc port=" + sourcePort + " address=" + mulitcastAddr +
            " caps=\\\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265\\\" " +
            "! rtpulpfecdec ! rtpjitterbuffer latency=200 ! rtph265depay ! h265parse ! " +
            "queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 ! " +
            "avdec_h265 ! videoconvert ! videorate ! video/x-raw,format=RGB,framerate=" + sourceFramerate + " ! " +
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
