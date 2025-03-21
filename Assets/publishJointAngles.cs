using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class publishJointAngles : MonoBehaviour
{
    [SerializeField] List<float> angles;

    public void Publish()
    {
        string msg = "set_joint_angles";
        msg += ";" + angles[0];
        msg += ";" + angles[1];
        msg += ";" + angles[2];
        msg += ";" + angles[3];
        msg += ";" + angles[4];
        msg += ";" + angles[5];
        TcpController.inst.Publish(msg);
    }
}
