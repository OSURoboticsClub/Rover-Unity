using System.Collections;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.Events;
public class AntennaControlStatus
{
    public TowerControlMode operating_mode = TowerControlMode.DISABLED;
    public float current_pan_deg = 0.0f;
    public float current_elevation_deg = 0.0f;
    public float current_pan_deg_sec = 0.0f;
    public float current_elevation_deg_sec = 0.0f;
    public float current_pan_setpoint = 0.0f;
    public float current_elevation_setpoint = 0.0f;
    public int errors = 0;

    public void updateFromControlStatusJson(JObject control_status)
    {
        operating_mode = (TowerControlMode) control_status["data"].Value<int>("operating_mode");
        current_pan_deg = control_status["data"].Value<float>("current_pan_deg");
        current_elevation_deg = control_status["data"].Value<float>("current_elevation_deg");
        current_pan_deg_sec = control_status["data"].Value<float>("current_pan_deg_sec");
        current_elevation_deg_sec = control_status["data"].Value<float>("current_elevation_deg_sec");
        current_pan_setpoint = control_status["data"].Value<float>("current_pan_setpoint");
        current_elevation_setpoint = control_status["data"].Value<float>("current_elevation_setpoint");
    }
}

public class TowerSubscriber : MonoBehaviour
{
    [SerializeField] TextAsset antennaControlStatus;
    [SerializeField] public bool debug = false;

    JObject lastMessage = null;

    public AntennaControlStatus currentStatus = new AntennaControlStatus();
    public UnityEvent<AntennaControlStatus> onAntennaControlStatusUpdate = new UnityEvent<AntennaControlStatus>();
    


    // Start is called before the first frame update
    void Start()
    {
        UdpController.inst.ConfigureSubscription("/gs_tower_control/status/", "AntennaControlStatus");
    }

    // Update is called once per frame
    void Update()
    {
        JObject newMessage = UdpController.inst.GetLatestMessage("/gs_tower_control/status/");
        if(newMessage != lastMessage)
        {
            currentStatus.updateFromControlStatusJson(newMessage);
            lastMessage = newMessage;
            onAntennaControlStatusUpdate.Invoke(currentStatus);
        }
        
    }
}
