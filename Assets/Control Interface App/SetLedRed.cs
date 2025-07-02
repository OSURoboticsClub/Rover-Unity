using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetLedRed : MonoBehaviour
{
    public void SetRed()
    {
        TcpController.inst.Publish("autonomous_LED/color;0");
    }
}
