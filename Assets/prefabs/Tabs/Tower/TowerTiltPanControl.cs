using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using System.Linq;


public class TowerTiltPanControl : MonoBehaviour
{
    [SerializeField] private TowerSubscriber towerSubscriber;
    [SerializeField] private TextAsset antennaControlManualInput;
    [Header("Text Objects")]
    [SerializeField] private TMP_Text currentPanText;
    [SerializeField] private TMP_Text currentTiltText;
    [SerializeField] private TMP_InputField panSetpointText;
    [SerializeField] private TMP_InputField tiltSetpointText;

    [Header("Buttons")]
    [SerializeField] private Button incrementPan;
    [SerializeField] private Button decrementPan;
    [SerializeField] private Button incrementTilt;
    [SerializeField] private Button decrementTilt;
    private List<Button> buttons = new List<Button>();

    private void AntennaControlServiceCall(AntennaControlStatus status)
    {
        JObject msg = JObject.Parse(antennaControlManualInput.text);

        msg["topic"]="/gs_tower_control/manual_control_input";
        msg["data"]["elevation_deg"] = status.current_elevation_setpoint;
        msg["data"]["pan_deg"] = status.current_pan_setpoint;

        UdpController.inst.PublishMessage(msg.ToString());
    }

    // Start is called before the first frame update
    void Start()
    {   
        towerSubscriber.onAntennaControlStatusUpdate.AddListener(antennaControlStatusUpdate);

        panSetpointText.onEndEdit.AddListener(onEditPan);
        tiltSetpointText.onEndEdit.AddListener(onEditTilt);

        incrementPan.onClick.AddListener(()=> incrementPanFun(0.1f));
        decrementPan.onClick.AddListener(()=> incrementPanFun(-0.1f));
        incrementTilt.onClick.AddListener(()=> incrementTiltFun(0.1f));
        decrementTilt.onClick.AddListener(()=> incrementTiltFun(-0.1f));

        buttons.Add(incrementPan);
        buttons.Add(decrementPan);
        buttons.Add(incrementTilt);
        buttons.Add(decrementTilt);

        antennaControlStatusUpdate(towerSubscriber.currentStatus);
    }

    void onEditPan(string newText)
    {
        if (float.TryParse(newText, out float result))
        {
            towerSubscriber.currentStatus.current_pan_setpoint = result;
            AntennaControlServiceCall(towerSubscriber.currentStatus);
            towerSubscriber.onAntennaControlStatusUpdate.Invoke(towerSubscriber.currentStatus);
        }
    }
    void onEditTilt(string newText)
    {
        if (float.TryParse(newText, out float result))
        {
            towerSubscriber.currentStatus.current_elevation_setpoint = result;
            AntennaControlServiceCall(towerSubscriber.currentStatus);
            towerSubscriber.onAntennaControlStatusUpdate.Invoke(towerSubscriber.currentStatus);
        }
    }
    void incrementPanFun(float amount)
    {
        towerSubscriber.currentStatus.current_pan_setpoint += amount;
        panSetpointText.text = towerSubscriber.currentStatus.current_pan_setpoint.ToString();
        AntennaControlServiceCall(towerSubscriber.currentStatus);
        towerSubscriber.onAntennaControlStatusUpdate.Invoke(towerSubscriber.currentStatus);
    }
    void incrementTiltFun(float amount)
    {
        towerSubscriber.currentStatus.current_elevation_setpoint += amount;
        tiltSetpointText.text = towerSubscriber.currentStatus.current_elevation_setpoint.ToString();
        AntennaControlServiceCall(towerSubscriber.currentStatus);
        towerSubscriber.onAntennaControlStatusUpdate.Invoke(towerSubscriber.currentStatus);
    }

    public void antennaControlStatusUpdate(AntennaControlStatus newStatus)
    {
        Debug.Log("YIPEEEE" + newStatus.operating_mode);
        if(newStatus.operating_mode == TowerControlMode.MANUAL_CONTROL)
        {

            panSetpointText.interactable = true;
            tiltSetpointText.interactable = true;
            foreach(Button button in buttons)
            {
                button.interactable = true;
            }
        } else
        {
            panSetpointText.interactable = false;
            tiltSetpointText.interactable = false;
            foreach(Button button in buttons)
            {
                button.interactable = false;
            }
            if(!panSetpointText.isFocused)
            {
                panSetpointText.text = newStatus.current_pan_setpoint.ToString();
                tiltSetpointText.text = newStatus.current_elevation_setpoint.ToString();
            }
        }
        
        currentTiltText.text = newStatus.current_elevation_deg.ToString()+ "°";
        currentPanText.text = newStatus.current_pan_deg.ToString() + "°";

    }
}
