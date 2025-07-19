using UnityEngine;
using UnityEngine.UI;
using ROS2;
using TMPro;
using UnityEngine.InputSystem;
using System.Collections;

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

    public Slider driveSpeedSlider;
    
    private Color selectedColor = Color.green;
    private Color unselectedColor = Color.red;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.Joy> joy_pub;
    private IPublisher<sensor_msgs.msg.Joy> joy2_pub;

    private float publishRate = 1f / 60f; // 60 Hz
    private Coroutine inputPublisherCoroutine;

    void Awake()
    {
        controls = new GameController();
        controls.Enable();
        controls.DriveControl.Enable();
        debugText = debugTextObject.GetComponent<TextMeshProUGUI>();

        ROS2UnityCore ros2UnityCore = new ROS2UnityCore();
        if (ros2UnityCore.Ok())
        {
            ros2Node = ros2UnityCore.CreateNode("ROS2UnityListenerNode");
            joy_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Joy>("joy");
            joy2_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Joy>("joy2");
        }
    }

    void Start()
    {
        debugText = debugTextObject.GetComponent<TextMeshProUGUI>();
        UpdateButtonColors();
        ros2Unity = GetComponent<ROS2UnityComponent>();
        driveSpeedSlider.value = 0.5f;
        driveSpeedSlider.minValue = 0f;
        driveSpeedSlider.maxValue = 1f;

        driveButton.onClick.AddListener(SetDriveMode);
        armButton.onClick.AddListener(SetArmMode);
        offButton.onClick.AddListener(SetOffMode);

        inputPublisherCoroutine = StartCoroutine(PublishInputsAtFixedRate());
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
                    HandleArm(
                        leftJoyValue, rightJoyValue,
                        triggerWest, triggerEast,
                        buttonSouth, buttonEast, buttonWest, buttonNorth,
                        dpadEast, dpadWest, dpadNorth, dpadSouth,
                        start, select, shoulderWest, shoulderEast
                    );
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

    void HandleDrive(Vector2 leftJoy, Vector2 rightJoy,
        float triggerWest, float triggerEast,
        float buttonSouth, float buttonEast, float buttonWest, float buttonNorth,
        float dpadEast, float dpadWest, float dpadNorth, float dpadSouth,
        float start, float select,
        float shoulderWest, float shoulderEast)
    {

        driveSpeedSlider.value += (triggerWest-triggerEast)*0.025f;

        string drive_msg = "command_control/ground_station_drive";
        drive_msg += ";" + (leftJoy.y != 0 || rightJoy.x != 0).ToString();
        drive_msg += ";false";
        drive_msg += ";" + leftJoy.y*driveSpeedSlider.value;
        drive_msg += ";" + rightJoy.x*driveSpeedSlider.value;
        UdpController.inst.PublishControl(drive_msg);
 
        string pan_tilt_msg = "tower/pan_tilt/control";
        pan_tilt_msg += ";" + (start == 1);
        pan_tilt_msg += ";" + ((buttonWest - buttonEast)+(shoulderWest-shoulderEast)) * 20;
        pan_tilt_msg += ";" + (buttonNorth - buttonSouth) * 20;
        pan_tilt_msg += ";false";
        pan_tilt_msg += ";false";
        UdpController.inst.PublishControl(pan_tilt_msg);
    }

    void HandleArm(
        Vector2 leftJoy, Vector2 rightJoy,
        float triggerWest, float triggerEast,
        float buttonSouth, float buttonEast, float buttonWest, float buttonNorth,
        float dpadEast, float dpadWest, float dpadNorth, float dpadSouth,
        float start, float select,
        float shoulderWest, float shoulderEast)
    {
        float[] axes = new float[] {
            leftJoy.x, leftJoy.y, triggerWest,
            rightJoy.x, rightJoy.y, triggerEast,
            (int)dpadEast - (int)dpadWest, (int)dpadNorth - (int)dpadSouth
        };

        int[] buttons = new int[] {
            (int)buttonSouth, (int)buttonEast, (int)buttonWest, (int)buttonNorth,
            (int)shoulderWest, (int)shoulderEast,
            (int)start, (int)select,
            0, 0, 0
        };

        sensor_msgs.msg.Joy msg = new sensor_msgs.msg.Joy();
        msg.Axes = axes;
        msg.Buttons = buttons;
        joy_pub.Publish(msg);
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
