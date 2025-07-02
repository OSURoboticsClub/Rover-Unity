using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class HeadingOffset : MonoBehaviour
{
    [SerializeField] TMP_InputField inp;
    public void Send()
    {
        float offset = float.Parse(inp.text);
        string message = $"imu/headingOffset;" + offset.ToString("F2");
        TcpController.inst.Publish(message);
    }
}
