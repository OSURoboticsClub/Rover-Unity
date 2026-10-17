using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;

using Unity.VisualScripting;

/// <summary>
/// An example class for calling SetBool service from Unity via ROS2
/// </summary>
public class clearCanError : MonoBehaviour
{

    public IEnumerator clearCan(int canNum)
    {
        JObject msg;
        if(canNum == 0)
            msg["service"] = "/can0_clear_can";
        else
            msg["service"] = "/can_arm_clear_can";

        msg["srvType"] = "/std_srvs/src/SetBool";
        msg["request"]["data"] = true;

        JObject response = await UdpController.inst.PublishClientReq(msg.ToString());

        Debug.Log($"[Sync] Got answer: success = {response["response"]["success"]}, message = {response["response"][message]}");
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
    }
}
