using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class KeyboardController : MonoBehaviour
{
    public Button p1;
    public Button p2;
    public Button p3;
    public Button p4;
    public Button p5;
    public Button p6;
    public Button p7;
    public Button p8;
    public Button p9;
    public Button p0;

    public Button addExecutePoseButton;
    public Button removePoseButton;

    private GameController controls;  
    private RobotArmController armController;
    private publishJointAngles jointPublisher;

    private Color occupiedColor = Color.green;
    private Color unoccupiedColor = Color.red;

    List<float> customPose0 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> customPose1 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> customPose2 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> customPose3 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> customPose4 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> customPose5 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    static private List<float> zeroPose = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    private bool executeArmPose = true;
    public bool sendArmCommand = false;

    void Awake()
    {
        controls = new GameController();
        controls.Enable();
        controls.KeyboardControl.Enable();
    }

    void Start()
    {
        armController = GetComponent<RobotArmController>();
        jointPublisher = GetComponent<publishJointAngles>();
    }

    void Update()
    {
        // Use triggered/WasPressedThisFrame() instead of ReadValue<float>() for single press detection
        bool enter = controls.KeyboardControl.Enter.triggered;
        bool zero = controls.KeyboardControl.zerokey.triggered;
        bool one = controls.KeyboardControl.onekey.triggered;
        bool two = controls.KeyboardControl.twokey.triggered;
        bool three = controls.KeyboardControl.threekey.triggered;
        bool four = controls.KeyboardControl.fourkey.triggered;
        bool five = controls.KeyboardControl.fivekey.triggered;
        bool six = controls.KeyboardControl.sixkey.triggered;
        bool seven = controls.KeyboardControl.sevenkey.triggered;
        bool eight = controls.KeyboardControl.eightkey.triggered;
        bool nine = controls.KeyboardControl.ninekey.triggered;

        bool space = controls.KeyboardControl.spacekey.triggered;
        bool backspace = controls.KeyboardControl.backspacekey.triggered;

        bool shift = controls.KeyboardControl.shiftkey.triggered;

        sendArmCommand = false;
        if(enter)
        {
            sendArmCommand = true;
        } 
        else if(zero)
        {
            handleCustomPose(customPose0,p0);
        } 
        else if(one)
        {
            jointPublisher.publish_preset_pose_0();
        } 
        else if(two)
        {
            jointPublisher.publish_preset_pose_1();
        } 
        else if(three)
        {
            jointPublisher.publish_preset_pose_2();
        } 
        else if(four)
        {
            jointPublisher.publish_preset_pose_3();
        } 
        else if(five)
        {
            jointPublisher.publish_preset_pose_4();
        } 
        else if(six)
        {
            jointPublisher.publish_preset_pose_5();
        } 
        else if(seven)
        {
            handleCustomPose(customPose3,p7);
        } 
        else if(eight)
        {
            handleCustomPose(customPose4,p8);
        } 
        else if(nine)
        {
            handleCustomPose(customPose5,p9);
        } 
        else if(space)
        {
            executeArmPose = true;
            addExecutePoseButton.image.color = occupiedColor;
            removePoseButton.image.color = unoccupiedColor;
        } 
        else if(backspace)
        {
            executeArmPose = false;
            addExecutePoseButton.image.color = unoccupiedColor;
            removePoseButton.image.color = occupiedColor;
        }
        else if(shift)
        {
            jointPublisher.nullSendAnglesProcess();
            armController.remove_vis();
        }
    }

    public void handleCustomPose(List<float> customPose, Button poseButton)
    {
        Debug.Log(executeArmPose);
        // Check if pose is not set yet (equals zero pose)
        if (customPose.SequenceEqual(zeroPose) && executeArmPose)
        {
            // Set the pose to current robot arm position
            poseButton.image.color = occupiedColor;
            SetPoseFromCurrentPosition(customPose);
            Debug.Log("Pose set from current arm position");
        }
        else if (executeArmPose)
        {
            // Pose is already set and we want to execute it
            jointPublisher.publish_custom_pose(customPose);
            Debug.Log("Executing custom pose");
        } 
        else 
        {
            poseButton.image.color = unoccupiedColor;

            // Reset pose to zero
            for (int i = 0; i < customPose.Count; i++)
            {
                customPose[i] = 0.0f;
            }
            Debug.Log("Pose reset to zero");
        }
    }

    private float GetSignedAngle(float angle)
    {
        return (angle > 180f) ? angle - 360f : angle;
    }


    // Helper method to set pose from current robot arm position
    private void SetPoseFromCurrentPosition(List<float> customPose)
    {
        customPose.Clear();
        
        for (int i = 0; i < 6 && i < armController.joints.Length; i++)
        {

            
            float angleRad = armController.trueAngles[i];


            
            customPose.Add(angleRad);
        }
    }

}