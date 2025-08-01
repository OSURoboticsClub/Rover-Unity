using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class publishJointAngles : MonoBehaviour
{
    RobotArmController robotArm;
    KeyboardController keyboardController;

    Coroutine currentCoroutine = null;

    bool allowArmControl = true;

    List<float> preset_pose_0 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> preset_pose_1 = new List<float> { 0.0f, -0.34f, -1.98968f, 0.0f, 0.785398f, 0.0f };
    List<float> preset_pose_2 = new List<float> { 0.0f, -0.698132f, -1.65806f, 0.0f, -0.785698f, 0.0f };
    List<float> preset_pose_3 = new List<float> { 0.17453f, 1.22173f, -2.61799f, 0.0f, -0.17453f, 0.0f };

    void Awake()
    {
        robotArm = FindObjectOfType<RobotArmController>();
        keyboardController = FindObjectOfType<KeyboardController>();
    }

    public void Publish(List<float> angles)
    {
        string msg = "set_joint_angles";
        for (int i = 0; i < angles.Count; i++)
        {
            msg += ";" + angles[i];
        }

        robotArm.visualize_goal(angles);

        // Stop previous coroutine if still waiting
        if (currentCoroutine != null)
        {
            StopCoroutine(currentCoroutine);
        }

        currentCoroutine = StartCoroutine(SendAngles(msg));
    }

    public void publish_preset_pose_0()
    {
        Debug.Log(currentCoroutine);
        Publish(preset_pose_0);
    }
    public void publish_preset_pose_1()
    {
        Debug.Log(currentCoroutine);

        Publish(preset_pose_1);
    }
    public void publish_preset_pose_2()
    {
        Debug.Log(currentCoroutine);

        Publish(preset_pose_2);
    }
    public void publish_preset_pose_3()
    {
        Debug.Log(currentCoroutine);
        Publish(preset_pose_3);
    }

    public void publish_custom_pose(List<float> customPose)
    {
        Publish(customPose);
    }

    public void nullSendAnglesProcess()
    {
        StopCoroutine(currentCoroutine);
        currentCoroutine = null;

    }
    public bool getArmControlStatus()
    {
        return allowArmControl;
    }

    IEnumerator SendAngles(string msg)
    {
        // Wait for confirmation
        yield return new WaitUntil(() => keyboardController.sendArmCommand);
        allowArmControl = false;

        TcpController.inst.Publish("joy2;0");
        yield return new WaitForSeconds(0.15f);

        TcpController.inst.Publish(msg);

        yield return new WaitForSeconds(7f);
        TcpController.inst.Publish("joy2;0");
        allowArmControl = true;
        currentCoroutine = null; // Clear coroutine reference
        robotArm.remove_vis();
    }
}
