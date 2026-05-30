using Newtonsoft.Json.Linq;
using UnityEngine;
using UnityEngine.UI;
using SpaceNavigatorDriver;

public class SpaceMouseRoverController : MonoBehaviour
{
    public enum ManualControlMode
    {
        Off,
        Drive,
        Arm
    }

    public enum SpaceMouseAxis
    {
        TranslationX,
        TranslationY,
        TranslationZ,
        RotationX,
        RotationY,
        RotationZ
    }

    [Header("Mode")]
    public ControllerManager modeSource;
    public ManualControlMode fallbackMode = ManualControlMode.Off;

    [Header("Speed")]
    public Slider driveSpeedSlider;
    public Slider armSpeedSlider;
    public float driveLinearScale = 1.0f;
    public float driveAngularScale = 1.0f;
    public float armScale = 1.0f;
    public float deadzone = 0.05f;
    public float publishRateHz = 30.0f;

    [Header("Drive Mapping")]
    public SpaceMouseAxis driveForwardAxis = SpaceMouseAxis.RotationX;
    public SpaceMouseAxis driveYawAxis = SpaceMouseAxis.RotationY;
    public bool invertDriveForward = false;
    public bool invertDriveYaw = false;

    [Header("Arm Mapping")]
    public Vector3 armLinearSigns = new Vector3(1.0f, 1.0f, 1.0f);
    public Vector3 armAngularSigns = new Vector3(1.0f, 1.0f, 1.0f);
    public float verticalTriggerMultiplier = 2.0f;
    public float wristRollZeroEpsilon = 0.0000000000001f;

    [Header("Debug")]
    public bool logInput = false;

    private bool wasDriveActive = false;
    private bool wasArmActive = false;
    private float nextPublishTime = 0.0f;

    private void Awake()
    {
        if (modeSource == null)
        {
            modeSource = GetComponent<ControllerManager>();
        }

#if UNITY_EDITOR
        if (Application.platform == RuntimePlatform.LinuxEditor && !SpaceNavigatorLinuxDriver.IsConnected)
        {
            SpaceNavigatorLinuxDriver.Instance.OnAfterAssemblyReload();
        }
#endif
    }

    private void Update()
    {
        if (publishRateHz <= 0.0f)
        {
            return;
        }

        if (Time.unscaledTime < nextPublishTime)
        {
            return;
        }

        nextPublishTime = Time.unscaledTime + (1.0f / publishRateHz);

        if (!TryReadSpaceMouse(out Vector3 translation, out Vector3 rotation))
        {
            return;
        }

        translation = ApplyDeadzone(translation);
        rotation = ApplyDeadzone(rotation);

        if (logInput)
        {
            Debug.Log($"SpaceMouse translation={translation} rotation={rotation}");
        }

        switch (CurrentMode())
        {
            case ManualControlMode.Drive:
                PublishDrive(translation, rotation);
                break;
            case ManualControlMode.Arm:
                PublishArm(translation, rotation);
                break;
            case ManualControlMode.Off:
                PublishDriveStopIfNeeded();
                PublishArmStopIfNeeded();
                break;
        }
    }

    private ManualControlMode CurrentMode()
    {
        if (modeSource == null)
        {
            return fallbackMode;
        }

        switch (modeSource.currentMode)
        {
            case ControllerManager.ControlMode.Drive:
                return ManualControlMode.Drive;
            case ControllerManager.ControlMode.Arm:
                return ManualControlMode.Arm;
            default:
                return ManualControlMode.Off;
        }
    }

    private bool TryReadSpaceMouse(out Vector3 translation, out Vector3 rotation)
    {
        translation = Vector3.zero;
        rotation = Vector3.zero;

#if UNITY_EDITOR
        if (Application.platform == RuntimePlatform.LinuxEditor)
        {
            if (SpaceNavigatorLinuxDriver.IsConnected)
            {
                translation = SpaceNavigatorLinuxDriver.Translation;
                rotation = SpaceNavigatorLinuxDriver.Rotation;
                return true;
            }
        }
#endif

        if (SpaceNavigatorHID.current == null)
        {
            return false;
        }

        translation = SpaceNavigatorHID.current.Translation.ReadValue();
        rotation = SpaceNavigatorHID.current.Rotation.ReadValue();
        return true;
    }

    private void PublishDrive(Vector3 translation, Vector3 rotation)
    {
        if (UdpController.inst == null)
        {
            return;
        }

        float linearX = ReadAxis(driveForwardAxis, translation, rotation);
        float angularZ = ReadAxis(driveYawAxis, translation, rotation);

        if (invertDriveForward)
        {
            linearX *= -1.0f;
        }

        if (invertDriveYaw)
        {
            angularZ *= -1.0f;
        }

        float sliderScale = driveSpeedSlider != null ? driveSpeedSlider.value : 1.0f;
        linearX *= driveLinearScale * sliderScale;
        angularZ *= driveAngularScale * sliderScale;

        bool hasInput = Mathf.Abs(linearX) > 0.0f || Mathf.Abs(angularZ) > 0.0f;
        if (!hasInput && !wasDriveActive)
        {
            return;
        }

        JObject driveMessage = new JObject
        {
            ["topic"] = "cmd_vel",
            ["msgType"] = "geometry_msgs/msg/Twist",
            ["data"] = new JObject
            {
                ["linear"] = new JObject
                {
                    ["x"] = linearX,
                    ["y"] = 0.0f,
                    ["z"] = 0.0f
                },
                ["angular"] = new JObject
                {
                    ["x"] = 0.0f,
                    ["y"] = 0.0f,
                    ["z"] = angularZ
                }
            }
        };

        UdpController.inst.PublishMessage(driveMessage.ToString());
        wasDriveActive = hasInput;
    }

    private void PublishArm(Vector3 translation, Vector3 rotation)
    {
        if (UdpController.inst == null)
        {
            return;
        }

        float sliderScale = armSpeedSlider != null ? armSpeedSlider.value : 1.0f;
        float scale = armScale * sliderScale;

        Vector3 linear = Vector3.Scale(translation, armLinearSigns) * scale;
        Vector3 angular = Vector3.Scale(rotation, armAngularSigns) * scale;

        bool hasInput = linear != Vector3.zero || angular != Vector3.zero;
        if (!hasInput && !wasArmActive)
        {
            return;
        }

        // ROS joy_to_servo maps this Xbox-style Joy array into:
        // linear.x/y/z and angular.x/y/z TwistStamped commands.
        float leftTrigger = Mathf.Clamp01(Mathf.Max(0.0f, linear.z) * verticalTriggerMultiplier);
        float rightTrigger = Mathf.Clamp01(Mathf.Max(0.0f, -linear.z) * verticalTriggerMultiplier);

        float[] axes = new float[]
        {
            -linear.x,                 // LEFT_STICK_X -> twist.linear.x after ROS-side inversion
            -linear.y,                 // LEFT_STICK_Y -> twist.linear.y after ROS-side inversion
            leftTrigger,               // LEFT_TRIGGER -> positive linear.z contribution
            -angular.y,                // RIGHT_STICK_X -> twist.angular.y after ROS-side inversion
            angular.x,                 // RIGHT_STICK_Y -> twist.angular.x
            rightTrigger,              // RIGHT_TRIGGER -> negative linear.z contribution
            angular.z + wristRollZeroEpsilon, // D_PAD_X -> twist.angular.z
            0.0f                       // D_PAD_Y, unused for 6DOF IK motion
        };

        int[] buttons = new int[]
        {
            0, 0, 0, 0,
            0, 0,
            0, 0,
            0, 0, 0, 0, 0
        };

        JObject joyMessage = new JObject
        {
            ["topic"] = "joy",
            ["msgType"] = "sensor_msgs/msg/Joy",
            ["data"] = new JObject
            {
                ["header"] = new JObject
                {
                    ["stamp"] = new JObject
                    {
                        ["sec"] = 0,
                        ["nanosec"] = 0
                    },
                    ["frame_id"] = ""
                },
                ["axes"] = new JArray(axes),
                ["buttons"] = new JArray(buttons)
            }
        };

        UdpController.inst.PublishMessage(joyMessage.ToString());
        wasArmActive = hasInput;
    }

    private void PublishDriveStopIfNeeded()
    {
        if (!wasDriveActive)
        {
            return;
        }

        PublishDrive(Vector3.zero, Vector3.zero);
    }

    private void PublishArmStopIfNeeded()
    {
        if (!wasArmActive)
        {
            return;
        }

        PublishArm(Vector3.zero, Vector3.zero);
    }

    private float ReadAxis(SpaceMouseAxis axis, Vector3 translation, Vector3 rotation)
    {
        switch (axis)
        {
            case SpaceMouseAxis.TranslationX:
                return translation.x;
            case SpaceMouseAxis.TranslationY:
                return translation.y;
            case SpaceMouseAxis.TranslationZ:
                return translation.z;
            case SpaceMouseAxis.RotationX:
                return rotation.x;
            case SpaceMouseAxis.RotationY:
                return rotation.y;
            case SpaceMouseAxis.RotationZ:
                return rotation.z;
            default:
                return 0.0f;
        }
    }

    private Vector3 ApplyDeadzone(Vector3 value)
    {
        return new Vector3(
            ApplyDeadzone(value.x),
            ApplyDeadzone(value.y),
            ApplyDeadzone(value.z));
    }

    private float ApplyDeadzone(float value)
    {
        return Mathf.Abs(value) < deadzone ? 0.0f : value;
    }
}
