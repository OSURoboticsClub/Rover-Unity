using UnityEngine;
using System.Diagnostics;

public class startTCPRelay : MonoBehaviour
{


    private Process tcpRelayProcess;

    public void LaunchTCPRelay()
    {
        if (tcpRelayProcess == null)
        {
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.FileName = "/bin/bash";


string command = $"source /opt/ros/humble/setup.bash && cd ~/Rover-Unity/ros2_ws && . install/setup.bash && ros2 run py_pubsub tcp_Relay";
            startInfo.Arguments = $"-c \"{command}\"";

            startInfo.UseShellExecute = false;
            startInfo.RedirectStandardOutput = true;
            startInfo.RedirectStandardError = true;
            //startInfo.CreateNoWindow = true; 
	    //startInfo.EnvironmentVariables["AMENT_PREFIX_PATH"] = "/opt/ros/humble";
	    //startInfo.EnvironmentVariables["ROS_DISTRO"] = "humble";
	   

            tcpRelayProcess = new Process();
            tcpRelayProcess.StartInfo = startInfo;
        
            
/*
            tcpRelayProcess.OutputDataReceived += (sender, e) => UnityEngine.Debug.Log("[stdout] " + e.Data);
            tcpRelayProcess.ErrorDataReceived += (sender, e) => UnityEngine.Debug.LogError("[stderr] " + e.Data);
*/
            tcpRelayProcess.Start();

            UnityEngine.Debug.Log("TCP Relay launched.");
        }
    }

    public void StopTCPRelay()
    {
        if (tcpRelayProcess != null && !tcpRelayProcess.HasExited)
        {
            tcpRelayProcess.Kill();
            
            tcpRelayProcess.WaitForExit();
            UnityEngine.Debug.Log("TCP Relay process stopped.");
            tcpRelayProcess = null;
        }
    }
}








