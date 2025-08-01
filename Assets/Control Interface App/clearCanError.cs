using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using ROS2;

using setBoolReq = std_srvs.srv.SetBool_Request;
using setBoolResp = std_srvs.srv.SetBool_Response;
using Unity.VisualScripting;

/// <summary>
/// An example class for calling SetBool service from Unity via ROS2
/// </summary>
public class clearCanError : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IClient<setBoolReq, setBoolResp> setBoolClientCan0;
    private IClient<setBoolReq, setBoolResp> setBoolClientCan1;

    private bool isRunning = false;
    private Task<setBoolResp> asyncTask;

    private std_srvs.srv.SetBool_Response response;

    public IEnumerator clearCan(int canNum)
    {
        while (!setBoolClientCan0.IsServiceAvailable())
        {
            yield return new WaitForSecondsRealtime(1);
        }

        setBoolReq request = new setBoolReq();
        request.Data = true;

        if (canNum == 0)
        {
            response = setBoolClientCan0.Call(request);
        }
        else
        {
            response = setBoolClientCan1.Call(request);
        }

        Debug.Log($"[Sync] Got answer: success = {response.Success}, message = {response.Message}");
    }

    // ✅ NEW METHODS FOR BUTTON CALLS
    public void CallClearCan0()
    {
        StartCoroutine(clearCan(0));
    }

    public void CallClearCan1()
    {
        StartCoroutine(clearCan(1));
    }

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityCan0ClearClient");
                setBoolClientCan0 = ros2Node.CreateClient<setBoolReq, setBoolResp>(
                    "/can0_clear_can"); // Change to your service name if different
                setBoolClientCan1 = ros2Node.CreateClient<setBoolReq, setBoolResp>(
                    "/can1_clear_can"); // Change to your service name if different
            }
        }
    }
}
