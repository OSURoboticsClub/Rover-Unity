using UnityEngine;
using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using TMPro;
using ROS2;

using Debug = UnityEngine.Debug;

using BoolReq = std_srvs.srv.SetBool_Request;
using BoolResp = std_srvs.srv.SetBool_Response;

public class GStreamerLauncher : MonoBehaviour
{
    public string portNum;
    private string sourcePort;
    private string sourceFramerate;
    private Process gStreamerProcess;
    public TMP_Dropdown dropdown;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private Dictionary<string, IClient<BoolReq, BoolResp>> cameraClients;

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

    private IEnumerator ToggleCameraThenLaunch(CameraEntry entry)
    {
        if (!cameraClients.TryGetValue(entry.serviceNamespace, out IClient<BoolReq, BoolResp> client))
        {
            Debug.LogError($"No ROS2 client found for camera '{entry.serviceNamespace}'.");
            yield break;
        }

        if (activeEntry != null && activeEntry.serviceNamespace != entry.serviceNamespace)
        {
            yield return SetCameraState(activeEntry, false);
        }

        while (!client.IsServiceAvailable())
        {
            Debug.Log($"Waiting for /{entry.serviceNamespace}/toggle_camera service...");
            yield return new WaitForSecondsRealtime(1);
        }

        Task<BoolResp> task = CallToggleCameraAsync(client, true);
        yield return new WaitUntil(() => task.IsCompleted);
        BoolResp response = task.Result;
        Debug.Log($"[{entry.serviceNamespace}] toggle_camera response: success = {response.Success}, message = {response.Message}");

        if (response.Success)
        {
            activeEntry = entry;
            LaunchGStreamer();
        }
        else
        {
            Debug.LogWarning($"Camera toggle failed for {entry.serviceNamespace}: {response.Message}");
        }
    }

    private IEnumerator SetCameraState(CameraEntry entry, bool state)
    {
        if (!cameraClients.TryGetValue(entry.serviceNamespace, out IClient<BoolReq, BoolResp> client))
        {
            yield break;
        }

        while (!client.IsServiceAvailable())
        {
            yield return new WaitForSecondsRealtime(1);
        }

        Task<BoolResp> task = CallToggleCameraAsync(client, state);
        yield return new WaitUntil(() => task.IsCompleted);
        BoolResp response = task.Result;
        Debug.Log($"[{entry.serviceNamespace}] toggle_camera({state}) response: success = {response.Success}, message = {response.Message}");
    }

    private Task<BoolResp> CallToggleCameraAsync(IClient<BoolReq, BoolResp> client, bool state)
    {
        BoolReq request = new BoolReq();
        request.Data = state;
        return client.CallAsync(request);
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
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("No ROS2UnityComponent found on this GameObject - add one.");
        }
        else if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode($"ROS2UnityCameraToggleClient_{Mathf.Abs(GetInstanceID())}");
                cameraClients = new Dictionary<string, IClient<BoolReq, BoolResp>>();
                foreach (CameraEntry entry in cameraPortMap.Values)
                {
                    if (cameraClients.ContainsKey(entry.serviceNamespace))
                    {
                        continue;
                    }

                    try
                    {
                        cameraClients[entry.serviceNamespace] = ros2Node.CreateClient<BoolReq, BoolResp>(
                            $"/{entry.serviceNamespace}/toggle_camera");
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogWarning($"Could not create toggle_camera client for '{entry.serviceNamespace}': {e.Message}");
                    }
                }
                Debug.Log($"Created {cameraClients.Count}/{cameraPortMap.Count} camera toggle clients.");
            }
        }
        else
        {
            Debug.LogError("ros2Unity.Ok() returned false - ROS2 not initialized.");
        }
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
            StartCoroutine(ToggleCameraThenLaunch(entry));
        }
        else
        {
            Debug.LogWarning("No port mapping found for dropdown index: " + index);
        }
    }

    public void LaunchGStreamer()
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
