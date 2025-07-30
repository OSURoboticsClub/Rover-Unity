using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using ROS2;
using System;
using example_interfaces.msg;
using TMPro;

public class sensorDataROS : MonoBehaviour
{
    private ISubscription<std_msgs.msg.Float32MultiArray> sensor_sub;

    public GameObject hydrogenTextObject;
    public GameObject ozoneTextObject;
    public GameObject geigerTextObject;

    private TextMeshProUGUI hydrogenText;
    private TextMeshProUGUI ozoneText;
    private TextMeshProUGUI geigerText;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node sensor_node;

    // Shared sensor data, updated by ROS2 callback
    private float hydrogen = 0f;
    private float ozone = 0f;
    private float geiger = 0f;
    private bool newDataAvailable = false;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("ROS2UnityComponent not found!");
            return;
        }

        hydrogenText = hydrogenTextObject?.GetComponent<TextMeshProUGUI>();
        ozoneText = ozoneTextObject?.GetComponent<TextMeshProUGUI>();
        geigerText = geigerTextObject?.GetComponent<TextMeshProUGUI>();
    }

    void Update()
    {
        if (sensor_node == null && ros2Unity.Ok())
        {
            sensor_node = ros2Unity.CreateNode("unity_sensor_sub");
            sensor_sub = sensor_node.CreateSubscription<std_msgs.msg.Float32MultiArray>(
                "scimech/data",
                msg =>
                {
                   
                        hydrogen = msg.Data[0];
                        ozone = msg.Data[1];
                        geiger = msg.Data[2];
                        newDataAvailable = true;
                    
                });
        }

        if (newDataAvailable)
        {
            if (hydrogenText != null) hydrogenText.text = $"H2: {hydrogen:F2}";
            if (ozoneText != null) ozoneText.text = $"O3: {ozone:F2}";
            if (geigerText != null) geigerText.text = $"Geiger: {geiger:F2}";
            newDataAvailable = false;
        }
    }
}
