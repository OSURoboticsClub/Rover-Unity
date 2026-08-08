using UnityEngine;
using Newtonsoft.Json.Linq;

/// <summary>
/// Calls the move_perp_to_plane Trigger service on the perp_to_plane node, which
/// commands the arm to align perpendicular to the most prominent plane in the point cloud.
///
/// Uses the custom UDP ROS bridge (via UdpController) instead of the built-in ROS2
/// bridge. The call is fire-and-forget: the request is sent without waiting for or
/// decoding a response. Assign a std_srvs/srv/Trigger service JSON template
/// (Assets/Templates/std_srvs/srv/Trigger.json) to triggerServiceJson in the inspector.
/// </summary>
public class PerpToPlane : MonoBehaviour
{
    [SerializeField] private TextAsset triggerServiceJson;

    // ROS service name the UDP bridge will forward the call to.
    private const string ServiceName = "move_perp_to_plane";

    // Hook this up to the button's OnClick event
    public void CallPerpToPlane()
    {
        Debug.Log("[PerpToPlane] CallPerpToPlane invoked.");

        if (triggerServiceJson == null)
        {
            Debug.LogError("[PerpToPlane] triggerServiceJson TextAsset is not assigned in the inspector.");
            return;
        }

        if (UdpController.inst == null)
        {
            Debug.LogError("[PerpToPlane] UdpController.inst is null - no UDP controller in the scene.");
            return;
        }

        JObject msg = JObject.Parse(triggerServiceJson.text);
        msg["service"] = ServiceName;
        // Trigger has an empty request, so nothing to populate.

        Debug.Log("[PerpToPlane] Sending move_perp_to_plane request over UDP bridge (no wait).");
        UdpController.inst.SendClientReq(msg.ToString());
    }
}
