using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using System;
using Newtonsoft.Json.Linq;
using Unity.VisualScripting;

public class RobotArmController : MonoBehaviour
{
    public static RobotArmController inst;
    public Transform[] joints;
    public Transform[] vis_joints;

    [SerializeField] float angle;
    [SerializeField] List<float> offsets;
    [SerializeField] public List<int> multipliers;

    // Joint names as they appear in the JointState message, in the same order as joints[].
    // Configurable in the Inspector — no code change needed if names change.
    [SerializeField] string[] jointNames = new string[]
    {
        "rover_arm_base_joint",
        "rover_arm_shoulder_joint",
        "rover_arm_elbow_pitch_joint",
        "rover_arm_elbow_roll_joint",
        "rover_arm_wrist_pitch_joint",
        "rover_arm_wrist_roll_joint"
    };

    public float[] trueAngles = { 0f, 0f, 0f, 0f, 0f, 0f };

    public GameObject rover_arm_vis;

    void Start()
    {
        inst = this;
        StartCoroutine(joint_state_subscription());
        UdpController.inst.ConfigureSubscription("joint_states", "sensor_msgs/msg/JointState");
        Debug.Log("Starting joint controller");
    }

    IEnumerator joint_state_subscription()
    {
        // Rotation axis per joint — true = Y axis, false = Z axis.
        // Matches the original Quaternion.Euler layout.
        bool[] rotateY = { true, false, false, true, false, true };

        while (true)
        {
            JObject jointMessage = UdpController.inst.GetLatestMessage("joint_states");
            if (jointMessage != null)
            {
                if (joints == null || joints.Length < 6)
                {
                    Debug.LogError("[RobotArmController] joints array not set or has fewer than 6 entries.");
                    yield break;
                }

                JArray names     = (JArray)jointMessage["data"]["name"];
                JArray positions = (JArray)jointMessage["data"]["position"];

                if (names == null || positions == null)
                {
                    yield return new WaitForSeconds(0.03f);
                    continue;
                }

                // Build a name → array-index lookup from this message.
                // Rebuilt each tick so it stays correct if joint ordering ever changes.
                var nameToIndex = new Dictionary<string, int>(names.Count);
                for (int i = 0; i < names.Count; i++)
                    nameToIndex[names[i].Value<string>()] = i;

                for (int j = 0; j < jointNames.Length && j < joints.Length; j++)
                {
                    if (!nameToIndex.TryGetValue(jointNames[j], out int idx))
                    {
                        Debug.LogWarning($"[RobotArmController] '{jointNames[j]}' not found in JointState message.");
                        continue;
                    }

                    float rad = positions[idx].Value<float>();
                    float deg = rad * Mathf.Rad2Deg * multipliers[j] + offsets[j];

                    joints[j].localRotation = rotateY[j]
                        ? Quaternion.Euler(0, deg, 0)
                        : Quaternion.Euler(0, 0, deg);

                    trueAngles[j] = rad;
                }
            }
            yield return new WaitForSeconds(0.03f);
        }
    }

    public void Receive(string message)
    {
        if (message.Contains("nan")) return;

        var parts = message.Split(";");

        try
        {
        }
        catch (Exception e)
        {
            Debug.LogError("[RobotArmController]: " + e.Message + "\nOriginal msg: " + message);
        }
    }

    public void visualize_goal(List<float> angles)
    {
        Vector3[] axes = new Vector3[] { Vector3.up, Vector3.forward, Vector3.forward, Vector3.up, Vector3.forward, Vector3.up };
        for (int i = 0; i < 6; i++)
        {
            vis_joints[i].localRotation = Quaternion.AngleAxis(angles[i] * 57.2958f * multipliers[i], axes[i]);
        }
        rover_arm_vis.SetActive(true);
    }

    public void remove_vis()
    {
        rover_arm_vis.SetActive(false);
    }
}
