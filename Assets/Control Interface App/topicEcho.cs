using UnityEngine;
using System.Diagnostics;

public class topicEcho : MonoBehaviour
{

    
    public GameObject topicObject;
    private Process tcpRelatProcess;

    public void LaunchTopicEcho()
    {
        if (tcpRelatProcess == null)
        {
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.FileName = "/bin/bash";

	    string objectName = topicObject.name;
	    
            string command = $"source /opt/ros/humble/setup.bash && ros2 topic echo "+objectName;
            UnityEngine.Debug.Log(command);
            startInfo.Arguments = $"-c \"{command}\"";

            startInfo.UseShellExecute = false;
            startInfo.RedirectStandardOutput = true;
            startInfo.RedirectStandardError = true;
            //startInfo.CreateNoWindow = true; 
	    //startInfo.EnvironmentVariables["AMENT_PREFIX_PATH"] = "/opt/ros/humble";
	    //startInfo.EnvironmentVariables["ROS_DISTRO"] = "humble";
	   

            tcpRelatProcess = new Process();
            tcpRelatProcess.StartInfo = startInfo;
        
            

            tcpRelatProcess.OutputDataReceived += (sender, e) => UnityEngine.Debug.Log("[stdout] " + e.Data);
            tcpRelatProcess.ErrorDataReceived += (sender, e) => UnityEngine.Debug.LogError("[stderr] " + e.Data);

            tcpRelatProcess.Start();

            UnityEngine.Debug.Log("Topic echo started");
        }
    }

    public void StopTopicEcho()
    {
        if (tcpRelatProcess != null && !tcpRelatProcess.HasExited)
        {
            tcpRelatProcess.Kill();
            
            tcpRelatProcess.WaitForExit();
            UnityEngine.Debug.Log("Topic echo stopped");
            tcpRelatProcess = null;
        }
    }
}








