using UnityEngine;
using System.Collections.Generic;
using System;


public class RobotArmController : MonoBehaviour
{
    public static RobotArmController inst;
    public Transform[] joints; 
    public Transform[] vis_joints;

    [SerializeField] float angle;
    [SerializeField] List<float> offsets;
    [SerializeField] public List<int> multipliers;

    public float[] trueAngles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    public GameObject rover_arm_vis;

    void Start()
    {
        inst = this;
    }
 
    public void Receive(string message){
        if(message.Contains("nan")) return;

        var parts = message.Split(";");

        try {
            joints[0].localRotation = Quaternion.Euler(0, float.Parse(parts[1]) * 57.2958f * multipliers[0] + offsets[0], 0);
            joints[1].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[2]) * 57.2958f * multipliers[1] + offsets[1]);
            joints[2].localRotation = Quaternion.Euler(0, float.Parse(parts[3]) * 57.2958f * multipliers[2] + offsets[2], 0);
            joints[3].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[4]) * 57.2958f * multipliers[3] + offsets[3]);
            joints[4].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[5]) * 57.2958f * multipliers[4] + offsets[4]);
            joints[5].localRotation = Quaternion.Euler(0, float.Parse(parts[6]) * 57.2958f * multipliers[5] + offsets[5], 0);

            trueAngles[0] = float.Parse(parts[1]);
            trueAngles[1] = float.Parse(parts[2]);
            trueAngles[3] = float.Parse(parts[3]);
            trueAngles[2] = float.Parse(parts[4]);
            trueAngles[4] = float.Parse(parts[5]);
            trueAngles[5] = float.Parse(parts[6]);

        }
        catch (Exception e){
            Debug.LogError("[RobotArmController]: " + e.Message + "\nOriginal msg: " + message);
            return;
        }
    }

    public void visualize_goal(List<float> angles)

    {   
        
        Vector3[] axes = new Vector3[] {Vector3.up, Vector3.forward, Vector3.forward, Vector3.up, Vector3.forward, Vector3.up};
        for(int i = 0; i < 6; i++)
        {
            vis_joints[i].localRotation = Quaternion.AngleAxis(angles[i]* 57.2958f * multipliers[i], axes[i]);
        }
        rover_arm_vis.SetActive(true);
        
    }
    public void remove_vis(){
        rover_arm_vis.SetActive(false);
    }
   

}
