using UnityEngine;
using System.Collections.Generic;


public class RobotArmController : MonoBehaviour
{
    public static RobotArmController inst;
    public Transform[] joints; // Assign your robot's joints in the Inspector
    [SerializeField] float angle;
    [SerializeField] List<float> offsets;
    [SerializeField] List<int> multipliers;

    void Start()
    {
        inst = this;
    }

    public void Receive(string message){
        var parts = message.Split(";");
        //Debug.Log("Received " + message);

        joints[0].localRotation = Quaternion.Euler(0, float.Parse(parts[1]) * 57.2958f * multipliers[0] + offsets[0], 0);
        joints[1].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[2]) * 57.2958f * multipliers[1] + offsets[1]);
        joints[2].localRotation = Quaternion.Euler(0, float.Parse(parts[3]) * 57.2958f * multipliers[2] + offsets[2], 0);
        joints[3].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[4]) * 57.2958f * multipliers[3] + offsets[3]);
        joints[4].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[5]) * 57.2958f * multipliers[4] + offsets[4]);
        joints[5].localRotation = Quaternion.Euler(0, float.Parse(parts[6]) * 57.2958f * multipliers[5] + offsets[5], 0);
    }
}
