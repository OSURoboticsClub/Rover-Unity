using UnityEngine;
using System.Collections.Generic;


public class RobotArmController : MonoBehaviour
{
    public static RobotArmController inst;
    public Transform[] joints; // Assign your robot's joints in the Inspector
    [SerializeField] float angle;
    [SerializeField] List<float> offsets;

    void Start()
    {
        inst = this;
    }

    public void Receive(string message){
        var parts = message.Split(";");
        Debug.Log("Received " + message);

        joints[0].localRotation = Quaternion.Euler(0, float.Parse(parts[1]) * 57.2958f + offsets[0], 0);
        joints[1].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[2]) * 57.2958f + offsets[1]);
        joints[2].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[3]) * 57.2958f + offsets[2]);
        joints[3].localRotation = Quaternion.Euler(0, float.Parse(parts[4]) * 57.2958f + offsets[3], 0);
        joints[4].localRotation = Quaternion.Euler(0, 0, float.Parse(parts[5]) * 57.2958f + offsets[4]);
        joints[5].localRotation = Quaternion.Euler(0, float.Parse(parts[6]) * 57.2958f + offsets[5], 0);
    }
}
