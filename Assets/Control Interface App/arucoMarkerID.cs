using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using ROS2;
using System;
using example_interfaces.msg;
using TMPro;

public class arucoMarkerID : MonoBehaviour
{
    private ISubscription<std_msgs.msg.Int32> aruco_sub_ir;
    private ISubscription<std_msgs.msg.Int32> aruco_sub_tower;
    private ISubscription<std_msgs.msg.Int32> aruco_sub_gripper;

    public GameObject arucoTextObject;
    public GameObject towerTextObject;
    public GameObject gripperTextObject;

    private TextMeshProUGUI arucoText;
    private TextMeshProUGUI towerText;
    private TextMeshProUGUI gripperText;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node aruco_node;

    // Values and update flags
    private int aruco_val_ir = -1;
    private int aruco_val_tower = -1;
    private int aruco_val_gripper = -1;

    private bool newDataIR = false;
    private bool newDataTower = false;
    private bool newDataGripper = false;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("ROS2UnityComponent not found!");
            return;
        }

        arucoText = arucoTextObject?.GetComponent<TextMeshProUGUI>();
        towerText = towerTextObject?.GetComponent<TextMeshProUGUI>();
        gripperText = gripperTextObject?.GetComponent<TextMeshProUGUI>();
    }

    void Update()
    {
        if (aruco_node == null && ros2Unity.Ok())
        {
            aruco_node = ros2Unity.CreateNode("unity_aruco_sub");

            aruco_sub_ir = aruco_node.CreateSubscription<std_msgs.msg.Int32>(
                "/infrared/aruco_id",
                msg =>
                {
                    aruco_val_ir = msg.Data;
                    newDataIR = true;
                });

            aruco_sub_tower = aruco_node.CreateSubscription<std_msgs.msg.Int32>(
                "/tower/aruco_id",
                msg =>
                {
                    aruco_val_tower = msg.Data;
                    newDataTower = true;
                });

            aruco_sub_gripper = aruco_node.CreateSubscription<std_msgs.msg.Int32>(
                "/gripper/aruco_id",
                msg =>
                {
                    aruco_val_gripper = msg.Data;
                    newDataGripper = true;
                });
        }

        if (newDataIR && arucoText != null)
        {
            arucoText.text = $"IR: {aruco_val_ir}";
            newDataIR = false;
        }

        if (newDataTower && towerText != null)
        {
            towerText.text = $"Tower: {aruco_val_tower}";
            newDataTower = false;
        }

        if (newDataGripper && gripperText != null)
        {
            gripperText.text = $"Gripper: {aruco_val_gripper}";
            newDataGripper = false;
        }
    }
}
