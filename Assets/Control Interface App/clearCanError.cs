using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json.Linq;

using Unity.VisualScripting;
using System.Globalization;

/// <summary>
/// An example class for calling SetBool service from Unity via ROS2
/// </summary>
public class clearCanError : MonoBehaviour
{
    // ✅ NEW METHODS FOR BUTTON CALLS
    public async Task CallClearCan0()
    {
        JObject msg = new JObject();
        msg["service"] = "/can0_clear_can";
        
        msg["srvType"] = "/std_srvs/src/SetBool";
        msg["request"]["data"] = true;

        JObject response = await UdpController.inst.PublishClientReq(msg.ToString());
        Debug.Log($"[Sync] Got answer: success = {response["response"]["success"]}, message = {response["response"]["message"]}");
    }

    public async Task CallClearCan1()
    {
        JObject msg = new JObject();
        msg["service"] = "/can_arm_clear_can";

        msg["srvType"] = "/std_srvs/src/SetBool";
        msg["request"]["data"] = true;

        JObject response = await UdpController.inst.PublishClientReq(msg.ToString());
        Debug.Log($"[Sync] Got answer: success = {response["response"]["success"]}, message = {response["response"]["message"]}");
    }

    void Start()
    {
    }
}
