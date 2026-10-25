using UnityEngine;
using UnityEngine.UI;
using TMPro;
using UnityEngine.InputSystem;
using System.Collections;
using Newtonsoft.Json.Linq;


public class ControllerManager : MonoBehaviour
{
    public enum ControlMode
    {
        Off,
        Drive,
        Arm
    }

    public GameObject debugTextObject; 
    private TextMeshProUGUI debugText;
    public ControlMode currentMode = ControlMode.Off;
    private GameController controls;  
    public Button driveButton;
    public Button armButton;
    public Button offButton;
    public Slider[] driveSpeedSlider;
    public Slider[] armSpeedSlider;
    public float driveSpeed = 0.5f;
    public float armSpeed = 0.5f;

    public double gimbalSpeedScale = 0.3;
    
    private Color selectedColor = Color.green;
    private Color unselectedColor = Color.red;

    
    private publishJointAngles jointPublisher;

    private bool useChassisPanTilt = false;
    private bool selectButtonPressed = false;
    private bool wasDriveActive = false;
    private bool wasArmActive = false;

    private float publishRate = 1f / 30f; // 30 Hz
    private Coroutine inputPublisherCoroutine;

    [SerializeField] private TextAsset driveMessageJson;

    [SerializeField] private TextAsset tiltMessageJson;

    [SerializeField] private TextAsset joyMessageJson;

    [SerializeField] private TextAsset uint8MessageJson;


    private JObject driveMessage;

    private JObject tiltMessage;

    private JObject joyMessage;

    private JObject joy2Message;

    void Awake()
    {
        controls = new GameController();
        controls.Enable();
        controls.DriveControl.Enable();
        debugText = debugTextObject.GetComponent<TextMeshProUGUI>();

        if (Application.platform == RuntimePlatform.WindowsPlayer ||
            Application.platform == RuntimePlatform.WindowsEditor) {
            return;
        }
    }

    void Start()
    {
        debugText = debugTextObject.GetComponent<TextMeshProUGUI>();
        UpdateButtonColors();
        foreach (Slider i in driveSpeedSlider) {
            i.value = 0.5f;
            i.minValue = 0f;
            i.maxValue = 1f;
        }
        
        foreach (Slider i in armSpeedSlider) {
            i.value = 0.5f;
            i.minValue = 0f;
            i.maxValue = 1f;
        }

        driveButton.onClick.AddListener(SetDriveMode);
        armButton.onClick.AddListener(SetArmMode);
        offButton.onClick.AddListener(SetOffMode);

        inputPublisherCoroutine = StartCoroutine(PublishInputsAtFixedRate());

        jointPublisher = GetComponent<publishJointAngles>();

    }

    IEnumerator PublishInputsAtFixedRate()
    {
        WaitForSeconds wait = new WaitForSeconds(publishRate);
        while (true)
        {
            Vector2 leftJoyValue = controls.DriveControl.leftJoy.ReadValue<Vector2>();
            Vector2 rightJoyValue = controls.DriveControl.rightJoy.ReadValue<Vector2>();

            float triggerEast = controls.DriveControl.triggerEast.ReadValue<float>();
            float triggerWest = controls.DriveControl.triggerWest.ReadValue<float>();

            float buttonEast = controls.DriveControl.buttonEast.ReadValue<float>();
            float buttonWest = controls.DriveControl.buttonWest.ReadValue<float>();
            float buttonNorth = controls.DriveControl.buttonNorth.ReadValue<float>();
            float buttonSouth = controls.DriveControl.buttonSouth.ReadValue<float>();
            float dpadEast = controls.DriveControl.dpadEast.ReadValue<float>();
            float dpadWest = controls.DriveControl.dpadWest.ReadValue<float>();
            float dpadNorth = controls.DriveControl.dpadNorth.ReadValue<float>();
            float dpadSouth = controls.DriveControl.dpadSouth.ReadValue<float>();
            float start = controls.DriveControl.start.ReadValue<float>();
            float select = controls.DriveControl.select.ReadValue<float>();
            float shoulderWest = controls.DriveControl.shoulderWest.ReadValue<float>();
            float shoulderEast = controls.DriveControl.shoulderEast.ReadValue<float>();

            switch (currentMode)
            {
                case ControlMode.Drive:
                    HandleDrive(
                        leftJoyValue, rightJoyValue,
                        triggerWest, triggerEast,
                        buttonSouth, buttonEast, buttonWest, buttonNorth,
                        dpadEast, dpadWest, dpadNorth, dpadSouth,
                        start, select, shoulderWest, shoulderEast);
                    break;
                case ControlMode.Arm:
                    if (jointPublisher.getArmControlStatus())
                    {
                        HandleArm(
                        leftJoyValue, rightJoyValue,
                        triggerWest, triggerEast,
                        buttonSouth, buttonEast, buttonWest, buttonNorth,
                        dpadEast, dpadWest, dpadNorth, dpadSouth,
                        start, select, shoulderWest, shoulderEast
                        );   
                    }
                    break;
            }

            yield return wait;
        }
    }

    void Update()
    {

        Vector2 leftJoyValue = controls.DriveControl.leftJoy.ReadValue<Vector2>();
        Vector2 rightJoyValue = controls.DriveControl.rightJoy.ReadValue<Vector2>();
        float triggerEast = controls.DriveControl.triggerEast.ReadValue<float>();
        float triggerWest = controls.DriveControl.triggerWest.ReadValue<float>();
        float buttonEast = controls.DriveControl.buttonEast.ReadValue<float>();
        float buttonWest = controls.DriveControl.buttonWest.ReadValue<float>();
        float buttonNorth = controls.DriveControl.buttonNorth.ReadValue<float>();
        float buttonSouth = controls.DriveControl.buttonSouth.ReadValue<float>();
        float dpadEast = controls.DriveControl.dpadEast.ReadValue<float>();
        float dpadWest = controls.DriveControl.dpadWest.ReadValue<float>();
        float dpadNorth = controls.DriveControl.dpadNorth.ReadValue<float>();
        float dpadSouth = controls.DriveControl.dpadSouth.ReadValue<float>();
        float start = controls.DriveControl.start.ReadValue<float>();
        float select = controls.DriveControl.select.ReadValue<float>();
        float shoulderWest = controls.DriveControl.shoulderWest.ReadValue<float>();
        float shoulderEast = controls.DriveControl.shoulderEast.ReadValue<float>();

        bool joystickEast = controls.DriveControl.joystickEastButton.triggered;

        //synchronize sliders
        foreach (Slider i in driveSpeedSlider)
        {
            if (i.value != driveSpeed)
            {
                driveSpeed = i.value;
                foreach (Slider j in driveSpeedSlider) {
                    j.value = driveSpeed;
                }
                break;
            }
        }

        foreach (Slider i in armSpeedSlider)
        {
            if (i.value != armSpeed)
            {
                armSpeed = i.value;
                foreach (Slider j in armSpeedSlider) {
                    j.value = armSpeed;
                }
                break;
            }
        }

        //Check joystick inputs
        if(joystickEast)
        {
            float[] axes = new float[]{
                0.0f
            };
            int[] buttons = new int[]{
                0,0,0,0,0,0,0,0,0,0,0
            };

            joy2Message = JObject.Parse(joyMessageJson.text);
            joy2Message["topic"] = "joy";
            joy2Message["data"]["axes"] = new JArray(axes);
            joy2Message["data"]["buttons"] = new JArray(buttons);

            string msg = joy2Message.ToString();
            UdpController.inst.PublishMessage(msg);

            // i dunno why this happens? TODO: ask someone why there are two button publishes
            buttons = new int[]{
                0,0,0,0,0,0,0,0,1,0,0
            };
            joy2Message["topic"] = "joy2";
            joy2Message["data"]["buttons"] = new JArray(buttons);
            msg = joy2Message.ToString();
            UdpController.inst.PublishMessage(msg);
        }

        string log = $@"
==== GameController Input ====
Left Stick:     X = {leftJoyValue.x:F2}, Y = {leftJoyValue.y:F2}
Right Stick:    X = {rightJoyValue.x:F2}, Y = {rightJoyValue.y:F2}
Triggers:       Left = {triggerWest:F2}, Right = {triggerEast:F2}

Buttons:
  A (South)     = {buttonSouth}
  B (East)      = {buttonEast}
  X (West)      = {buttonWest}
  Y (North)     = {buttonNorth}
  LB            = {shoulderWest}
  RB            = {shoulderEast}
  Start/Menu    = {start}
  Select/View   = {select}

D-Pad:
  East          = {dpadEast}
  West          = {dpadWest}
  North         = {dpadNorth}
  South         = {dpadSouth}
==============================
";

        debugText.text = log;
    }

    void SetOffMode()
    {
        currentMode = ControlMode.Off;
        Debug.Log("Switched to OFF mode");
        UpdateButtonColors();
    }

    void SetDriveMode()
    {
        currentMode = ControlMode.Drive;
        Debug.Log("Switched to DRIVE mode");
        UpdateButtonColors();
    }

    void SetArmMode()
    {
        currentMode = ControlMode.Arm;
        Debug.Log("Switched to ARM mode");
        UpdateButtonColors();
    }


    void HandleDrive(
        Vector2 leftJoy, Vector2 rightJoy,
        float triggerWest, float triggerEast,
        float buttonSouth, float buttonEast, float buttonWest, float buttonNorth,
        float dpadEast, float dpadWest, float dpadNorth, float dpadSouth,
        float start, float select,
        float shoulderWest, float shoulderEast
    )
    {   
        foreach (Slider i in driveSpeedSlider) {
            i.value += (triggerWest - triggerEast) * 0.025f;
        }
        driveSpeed = driveSpeedSlider[0].value;

        // Check if drive has input
        //bool driveHasInput = (leftJoy.y != 0 || rightJoy.x != 0);
        bool driveHasInput = true;

        // Publish drive command if there's input, or send stop command once when input stops
        if (driveHasInput || wasDriveActive)
        {
            
            driveMessage = JObject.Parse(driveMessageJson.text);
            driveMessage["topic"] = "cmd_vel";
            driveMessage["data"]["angular"]["z"] = rightJoy.x * driveSpeed * -1;
            driveMessage["data"]["linear"]["x"] = leftJoy.y * driveSpeed;
            string msg = driveMessage.ToString();
            UdpController.inst.PublishMessage(msg);

            wasDriveActive = driveHasInput;
        }

        // Handle select button toggle (only trigger on button press, not hold)
        if (select == 1 && !selectButtonPressed)
        {
            useChassisPanTilt = !useChassisPanTilt;
            selectButtonPressed = true;
        }
        else if (select == 0)
        {
            selectButtonPressed = false;
        }

        // Check if pan/tilt has input
        bool panTiltHasInput = (start == 1) ||
                            (buttonWest != 0 || buttonEast != 0) ||
                            (shoulderWest != 0 || shoulderEast != 0) ||
                            (buttonNorth != 0 || buttonSouth != 0) ||
                            (dpadEast != 0 || dpadWest != 0);

        // Only publish pan/tilt command if there's input
        if (panTiltHasInput)
        {
            // string panTiltPrefix = useChassisPanTilt ? "chassis/pan_tilt/control" : "tower/pan_tilt/control";

            // tiltMessage = JObject.Parse(tiltMessageJson.text);
            // tiltMessage["topic"] = panTiltPrefix;
            // tiltMessage["data"]["should_center"] = start==1;
            // tiltMessage["data"]["relative_pan_adjustment"] = ((buttonWest - buttonEast) + (shoulderWest - shoulderEast)) * 100;
            // tiltMessage["data"]["relative_tilt_adjustment"] = (buttonNorth - buttonSouth) * 100;

            // string msg = tiltMessage.ToString();
            // UdpController.inst.PublishMessage(msg);

            // 2526 Odrive pan tilt
            string odrivePanTiltPrefix = useChassisPanTilt ? "chassis_pan_tilt/control" : "tower_gimbal/control";
            tiltMessage = JObject.Parse(tiltMessageJson.text);
            tiltMessage["topic"] = odrivePanTiltPrefix;
            tiltMessage["data"]["go_home"] = start == 1;
            tiltMessage["data"]["is_angle"] = false;
            tiltMessage["data"]["stabalize"] = false;
            tiltMessage["data"]["yaw"] = gimbalSpeedScale * ((buttonWest - buttonEast) + (shoulderWest - shoulderEast)); 
            tiltMessage["data"]["pitch"] = gimbalSpeedScale * (buttonNorth - buttonSouth);
            tiltMessage["data"]["roll"] = gimbalSpeedScale * (dpadEast - dpadWest);

            string msg = tiltMessage.ToString();
            UdpController.inst.PublishMessage(msg);
            
            
        }
        
        if (dpadNorth == 1)
        {
            JObject msg = JObject.Parse(uint8MessageJson.text);
            msg["topic"] = "/tower/light/control";
            msg["msgType"] = "std_msgs/msg/UInt8";
            msg["data"]["data"] = 2;
            UdpController.inst.PublishMessage(msg.ToString());
        }
        else if (dpadSouth == 1)
        {
            JObject msg = JObject.Parse(uint8MessageJson.text);
            msg["topic"] = "/tower/light/control";
            msg["msgType"] = "std_msgs/msg/UInt8";
            msg["data"]["data"] = 0;
            UdpController.inst.PublishMessage(msg.ToString());
        }
        

    }

    void HandleArm(
        Vector2 leftJoy, Vector2 rightJoy,
        float triggerWest, float triggerEast,
        float buttonSouth, float buttonEast, float buttonWest, float buttonNorth,
        float dpadEast, float dpadWest, float dpadNorth, float dpadSouth,
        float start, float select,
        float shoulderWest, float shoulderEast)
    {
        // Check if arm has any input
        /*bool armHasInput = (leftJoy != Vector2.zero) ||
                        (rightJoy != Vector2.zero) ||
                        (triggerWest != 0) ||
                        (triggerEast != 0) ||
                        
                        (buttonEast != 0) ||
                        (buttonWest != 0) ||
                       
                        (dpadEast != 0) ||
                        (dpadWest != 0) ||
                        (dpadNorth != 0) ||
                        (dpadSouth != 0) ||
                        (start != 0) ||
                        (select != 0) ||
                        (shoulderWest != 0) ||
                        (shoulderEast != 0);*/
        bool armHasInput = true;
        foreach (Slider i in armSpeedSlider) {
            i.value += (buttonNorth-buttonSouth)*0.025f;
        }
        armSpeed = armSpeedSlider[0].value;
        // Only publish if there's input
        if (armHasInput || wasArmActive)
        {
            //The addition of the very small float in the wrist roll is to combat unexpected arm maneuvers.
            //More specifically, sending a 0 joint velocity for every joint after switching controllers and moving causes 
            //the arm to rapidly swing back to the pose the original controller put it in.
            float[] axes = new float[] {
                -leftJoy.x*armSpeed , leftJoy.y*armSpeed , triggerWest*armSpeed ,
                -rightJoy.x*armSpeed , rightJoy.y*armSpeed , triggerEast*armSpeed ,
                ((int)dpadEast*armSpeed  - (int)dpadWest*armSpeed+0.0000000000001f) , ((int)dpadNorth*armSpeed  - (int)dpadSouth*armSpeed) 
            };

            wasArmActive = armHasInput;

            int[] buttons = new int[] {
                (int)buttonSouth  , (int)buttonEast  , (int)buttonWest  , (int)buttonNorth  ,
                0  , 0  ,
                (int)start, (int)select,
                0, 0, (int)shoulderEast, (int)shoulderWest, (int)shoulderWest
            };


            joyMessage = JObject.Parse(joyMessageJson.text);
            joyMessage["topic"] = "joy";
            joyMessage["data"]["axes"] = new JArray(axes);
            joyMessage["data"]["buttons"] = new JArray(buttons);

            string msg = joyMessage.ToString();
            UdpController.inst.PublishMessage(msg);
        }
    }

    void UpdateButtonColors()
    {
        ColorBlock driveColors = driveButton.colors;
        ColorBlock armColors = armButton.colors;
        ColorBlock offColors = offButton.colors;

        if (currentMode == ControlMode.Drive)
        {
            driveColors.normalColor = selectedColor;
            armColors.normalColor = unselectedColor;
            offColors.normalColor = unselectedColor;
        }
        else if (currentMode == ControlMode.Arm)
        {
            driveColors.normalColor = unselectedColor;
            armColors.normalColor = selectedColor;
            offColors.normalColor = unselectedColor;
        }
        else
        {
            driveColors.normalColor = unselectedColor;
            armColors.normalColor = unselectedColor;
            offColors.normalColor = selectedColor;
        }

        driveButton.colors = driveColors;
        armButton.colors = armColors;
        offButton.colors = offColors;
    }
}
