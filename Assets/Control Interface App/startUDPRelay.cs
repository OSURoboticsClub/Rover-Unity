using UnityEngine;
using System.Diagnostics;

public class startUDPRelay : MonoBehaviour
{


    private Process udpRelayProcess;

    public void LaunchUDPRelay()
    {
        if (udpRelayProcess == null)
        {
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.FileName = "/bin/bash";


string command = $"source /opt/ros/humble/setup.bash && cd ~/Rover-Unity/ros2_ws && . install/setup.bash && ros2 run py_pubsub udp_Relay";
            startInfo.Arguments = $"-c \"{command}\"";

            startInfo.UseShellExecute = false;
            startInfo.RedirectStandardOutput = true;
            startInfo.RedirectStandardError = true;
            //startInfo.CreateNoWindow = true; 
	    //startInfo.EnvironmentVariables["AMENT_PREFIX_PATH"] = "/opt/ros/humble";
	    //startInfo.EnvironmentVariables["ROS_DISTRO"] = "humble";
	   

            udpRelayProcess = new Process();
            udpRelayProcess.StartInfo = startInfo;
        
            
/*
            udpRelayProcess.OutputDataReceived += (sender, e) => UnityEngine.Debug.Log("[stdout] " + e.Data);
            udpRelayProcess.ErrorDataReceived += (sender, e) => UnityEngine.Debug.LogError("[stderr] " + e.Data);
*/
            udpRelayProcess.Start();

            UnityEngine.Debug.Log("udp Relay launched.");
        }
    }

    public void StopudpRelay()
    {
        if (udpRelayProcess != null && !udpRelayProcess.HasExited)
        {
            udpRelayProcess.Kill();
            
            udpRelayProcess.WaitForExit();
            UnityEngine.Debug.Log("udp Relay process stopped.");
            udpRelayProcess = null;
        }
    }
}








