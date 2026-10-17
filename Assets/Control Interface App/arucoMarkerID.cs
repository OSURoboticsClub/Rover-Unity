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
    private TextMeshProUGUI gripperText

    // Values and update flags
    private int aruco_val_ir = -1;
    private int aruco_val_tower = -1;
    private int aruco_val_gripper = -1;

    private bool newDataIR = false;
    private bool newDataTower = false;
    private bool newDataGripper = false;

    void Start()
    {

        arucoText = arucoTextObject?.GetComponent<TextMeshProUGUI>();
        towerText = towerTextObject?.GetComponent<TextMeshProUGUI>();
        gripperText = gripperTextObject?.GetComponent<TextMeshProUGUI>();

        UdpController.inst.ConfigureSubscription("/infared/aruco_id","int32");
        UdpController.inst.ConfigureSubscription("/tower/aruco_id","int32");
        UdpController.inst.ConfigureSubscription("/gripper/aruco_id","int32");
    }


    void Update()
    {
        
        JObject msg;

        JObject message = UdpController.inst.GetLatestMessage("/infared/aruco_id")
        if (message.HasValues()){ //using HasValues() to check for update.
            arucoText.text = $"IR: {(int)message["Data"]}";
        }
        message = UdpController.inst.GetLatestMessage("/tower/aruco_id")
        if (message.HasValues()){
            towerText.text = $"Tower: {(int)message["Data"]}";
        }
        message = UdpController.inst.GetLatestMessage("/gripper/aruco_id")
        if (message.HasValues()){
            gripperText.text = $"Gripper: {(int)message["Data"]}";
        }

        /*
        Old system using ros2 for unity.
        I left it here because there is a 33.333% chance I messed up.
        If you know both systems better than my dumbass, please review it.
        */
        /*
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
        */
    }
}