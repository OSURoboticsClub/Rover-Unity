using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using TMPro;
using UnityEditor.Compilation;
using UnityEngine;

enum MechanismCommand
{
    SEND_MORSE = 0,
    TOGGLE_SOLENOID = 1,
    SERVO_HOME = 2,
    SERVO_XLR = 3,
    SERVO_MORSE = 4
}

public class HeistMechController : MonoBehaviour
{
    [SerializeField] private TextAsset heistServiceJson;

    [SerializeField] private GameObject statusTextObject;

    [SerializeField] private GameObject morseInputBoxGameObject;

    private TextMeshProUGUI statusText = null;

    private TMP_InputField morseInputText = null;

    private async Task HeistServiceCall(MechanismCommand cmd, string message = null)
    {
        JObject msg = JObject.Parse(heistServiceJson.text);

        msg["service"]="heist_mech";
        msg["request"]["command"] = (int)cmd;
        msg["request"]["message"] = (message == null) ? "" : message;

        await UdpController.inst.PublishClientReq(msg.ToString(), "heist_svc");
    }

    public void SendMorseMessage()
    {
        if (morseInputText != null && morseInputText.text != "") {
            Debug.Log("Attempting to send morse code message \"" + morseInputText.text + "\"");
            HeistServiceCall(MechanismCommand.SEND_MORSE, morseInputText.text);
        }
        else
        {
            if (statusText != null)
            {
                statusText.text = "Please enter a mesage to send.";
            }
        }
    }

    public void SetMechanismExtended(bool extended)
    {
        if (extended)
            HeistServiceCall(MechanismCommand.SERVO_XLR);
        else
            HeistServiceCall(MechanismCommand.SERVO_HOME);
    }

    public void ToggleSolenoid()
    {
        HeistServiceCall(MechanismCommand.TOGGLE_SOLENOID);
    }

    private void Awake()
    {
        statusText = statusTextObject?.GetComponent<TextMeshProUGUI>();
        morseInputText = morseInputBoxGameObject?.GetComponent<TMP_InputField>();
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        JObject result = UdpController.inst.GetLatestServiceResponse("heist_srv");
        if (statusText != null && result != null)
        {
            statusText.text = result["response"]["result_msg"].ToString();
        }
    }
}
