using System.Collections;
using UnityEngine;
using ROS2;

using TriggerReq = std_srvs.srv.Trigger_Request;
using TriggerResp = std_srvs.srv.Trigger_Response;

/// <summary>
/// Calls the move_perp_to_plane Trigger service on the perp_to_plane node, which
/// commands the arm to align perpendicular to the most prominent plane in the point cloud.
/// </summary>
public class PerpToPlane : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IClient<TriggerReq, TriggerResp> perpToPlaneClient;

    private TriggerResp response;

    public IEnumerator CallPerpToPlaneCoroutine()
    {
        if (perpToPlaneClient == null)
        {
            Debug.LogError("[PerpToPlane] perpToPlaneClient is null - ROS2UnityComponent was missing or not Ok() in Start().");
            yield break;
        }

        while (!perpToPlaneClient.IsServiceAvailable())
        {
            Debug.Log("[PerpToPlane] Waiting for move_perp_to_plane service...");
            yield return new WaitForSecondsRealtime(1);
        }

        TriggerReq request = new TriggerReq();

        response = perpToPlaneClient.Call(request);

        Debug.Log($"[PerpToPlane] Got answer: success = {response.Success}, message = {response.Message}");
    }

    // Hook this up to the button's OnClick event
    public void CallPerpToPlane()
    {
        Debug.Log("[PerpToPlane] CallPerpToPlane invoked.");
        StartCoroutine(CallPerpToPlaneCoroutine());
    }

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("[PerpToPlane] No ROS2UnityComponent found on this GameObject - add one.");
            return;
        }
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityPerpToPlaneClient");
                perpToPlaneClient = ros2Node.CreateClient<TriggerReq, TriggerResp>(
                    "move_perp_to_plane");
                Debug.Log("[PerpToPlane] Created client for move_perp_to_plane service.");
            }
        }
        else
        {
            Debug.LogError("[PerpToPlane] ros2Unity.Ok() returned false - ROS2 not initialized.");
        }
    }
}
