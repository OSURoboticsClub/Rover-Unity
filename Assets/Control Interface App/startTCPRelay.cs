using UnityEngine;
using System.Diagnostics;

public class startTCPRelay : MonoBehaviour
{


    private Process tcpRelatProcess;

    public void LaunchTCPRelay()
    {
        if (tcpRelatProcess == null)
        {
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.FileName = "/bin/bash";


string command = $"source /opt/ros/humble/setup.bash && cd ~/Rover-Unity/ros2_ws && . install/setup.bash && ros2 run py_pubsub tcp_relay";
            startInfo.Arguments = $"-c \"{command}\"";

            startInfo.UseShellExecute = false;
            startInfo.RedirectStandardOutput = true;
            startInfo.RedirectStandardError = true;
            //startInfo.CreateNoWindow = true; 
	    //startInfo.EnvironmentVariables["AMENT_PREFIX_PATH"] = "/opt/ros/humble";
	    //startInfo.EnvironmentVariables["ROS_DISTRO"] = "humble";
	   

            tcpRelatProcess = new Process();
            tcpRelatProcess.StartInfo = startInfo;
        
            
/*
            tcpRelatProcess.OutputDataReceived += (sender, e) => UnityEngine.Debug.Log("[stdout] " + e.Data);
            tcpRelatProcess.ErrorDataReceived += (sender, e) => UnityEngine.Debug.LogError("[stderr] " + e.Data);
*/
            tcpRelatProcess.Start();

            UnityEngine.Debug.Log("TCP Relay launched.");
        }
    }

    public void StopTCPRelay()
    {
        if (tcpRelatProcess != null && !tcpRelatProcess.HasExited)
        {
            tcpRelatProcess.Kill();
            
            tcpRelatProcess.WaitForExit();
            UnityEngine.Debug.Log("TCP Relay process stopped.");
            tcpRelatProcess = null;
        }
    }
}








