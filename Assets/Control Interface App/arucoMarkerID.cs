
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


    public GameObject arucoTextObject;
   

    private TextMeshProUGUI arucoText;


    private ROS2UnityComponent ros2Unity;
    private ROS2Node aruco_node;

    // Shared sensor data, updated by ROS2 callback
    private int aruco_val = -1;
    private bool newDataAvailable = false;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("ROS2UnityComponent not found!");
            return;
        }

        arucoText = arucoTextObject?.GetComponent<TextMeshProUGUI>();

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
                   
                    if(msg.Data != -1)
                    {
                        aruco_val = msg.Data;
                        newDataAvailable = true;
                    }
                    
                });
            aruco_sub_tower = aruco_node.CreateSubscription<std_msgs.msg.Int32>(
                "/tower/aruco_id",
                msg =>
                {
                    if(msg.Data != -1)
                    {
                        aruco_val = msg.Data;
                        newDataAvailable = true;
                    }
                        
                    
                });
        }

        if (newDataAvailable)
        {
            if (arucoText != null) arucoText.text = $"Found: {aruco_val}";

            newDataAvailable = false;
        }
    }
}
