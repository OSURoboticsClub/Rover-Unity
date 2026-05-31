using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

// Reads the SpaceMouse via the Linux evdev interface (/dev/input/event*) and publishes
// drive (cmd_vel) and arm (joy) commands through the UDP bridge.
//
// Why we manage InputSystem here:
//   Unity's InputSystem calls EVIOCGRAB on every detected HID joystick.
//   EVIOCGRAB routes all kernel input events exclusively to that one fd — the joydev layer
//   (js0) and every other reader gets nothing.  We immediately remove the SpaceMouse from
//   InputSystem in Awake() so the grab is released, then read event3 ourselves.
public class SpaceMouseRoverController : MonoBehaviour
{
    public enum AxisChoice
    {
        TranslationX, TranslationY, TranslationZ,
        RotationX, RotationY, RotationZ
    }

    [Header("Mode")]
    public ControllerManager modeSource;
    public publishJointAngles armControl;   // optional — gates arm publish like ControllerManager

    [Header("Publish")]
    public float publishRateHz = 30f;

    [Header("Deadzone")]
    [Tooltip("Minimum normalised value (0–1) before a translation axis registers. " +
             "Raise this if small unintended translations leak through.")]
    public float translationDeadzone = 0.05f;
    [Tooltip("Minimum normalised value (0–1) before a rotation axis registers. " +
             "Raise this if small unintended rotations leak through.")]
    public float rotationDeadzone = 0.05f;

    [Header("Speed")]
    public Slider driveSpeedSlider;
    public Slider armSpeedSlider;
    public float driveLinearScale  = 1f;
    public float driveAngularScale = 1f;
    public float armScale          = 1f;

    [Header("Evdev Device")]
    public string explicitEventPath  = "";         // leave blank to auto-discover
    public string byIdNameContains   = "3Dconnexion";
    public bool   discoverFromProc   = true;
    public string procNameContains   = "3Dconnexion";
    public bool   discoverEventNodes = false;       // last-resort full scan
    public float  translationScale   = 1f / 350f;
    public float  rotationScale      = 1f / 350f;
    public float  rediscoveryIntervalSeconds = 5f;
    public int    reconnectDelayMs   = 2000;

    // Push puck forward = TranslationY (inverted — kernel reports negative when pushing away)
    // Twist puck = RotationZ → cmd_vel.angular.z
    [Header("Drive Mapping")]
    public AxisChoice driveForwardAxis = AxisChoice.TranslationY;
    public AxisChoice driveYawAxis     = AxisChoice.RotationZ;
    public bool invertDriveForward = true;
    public bool invertDriveYaw     = false;

    [Header("Arm Mapping")]
    public Vector3 armLinearSigns  = Vector3.one;
    public Vector3 armAngularSigns = Vector3.one;
    public float   verticalTriggerMultiplier = 2f;
    public float   wristRollZeroEpsilon      = 1e-13f;

    // Maps physical SpaceMouse buttons to joy.buttons indices.
    // 10 = RB / 11 = LB (gripper open/close in the gamepad layout).
    [Header("Buttons — Arm Joy")]
    public int smButton0JoyIndex = 10;
    public int smButton1JoyIndex = 11;
    public int joyButtonCount    = 13;

    [Header("Buttons — Controller Toggle (Joy2)")]
    // Left SpaceMouse button toggles the arm controller via a joy2 message (button index 8).
    // Press once → engage SpaceMouse IK control.  Press again → release back to default.
    public int joy2ButtonIndex  = 8;
    public int joy2ButtonCount  = 11;

    [Header("Debug")]
    public bool logInput = false;

    // evdev constants
    private const ushort EvTypeKey = 0x01;
    private const ushort EvTypeRel = 0x02;
    private const ushort EvTypeAbs = 0x03;
    private const ushort AxisX     = 0x00;
    private const ushort AxisY     = 0x01;
    private const ushort AxisZ     = 0x02;
    private const ushort AxisRx    = 0x03;
    private const ushort AxisRy    = 0x04;
    private const ushort AxisRz    = 0x05;
    private const ushort BtnBase   = 0x100;  // BTN_0
    private const int    BtnCount  = 2;
    private const int    AxCount   = 6;

    private bool  _wasDriveActive    = false;
    private bool  _wasArmActive      = false;
    private float _nextPublishTime   = 0f;
    private float _nextDiscoveryTime = 0f;
    private bool  _joy2Active        = false;   // current controller toggle state

    private readonly float[] _axes       = new float[AxCount];
    private readonly int[]   _buttons    = new int[BtnCount];
    private readonly int[]   _prevButtons = new int[BtnCount];
    private readonly List<EvdevReader> _readers = new List<EvdevReader>();

    private string _inputError      = "";
    private string _lastLoggedError = "";

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        if (modeSource == null) modeSource = GetComponent<ControllerManager>();
        if (armControl  == null) armControl  = GetComponent<publishJointAngles>();

        // Release any EVIOCGRAB Unity's InputSystem may already hold, and keep
        // releasing if the device is re-added (e.g. after a USB replug).
        ReleaseFromInputSystem();
        InputSystem.onDeviceChange += OnInputDeviceChange;

        if (IsLinux()) StartReaders();
        else Debug.LogWarning("[SpaceMouse] evdev input only supported on Linux.");
    }

    private void OnDestroy()
    {
        InputSystem.onDeviceChange -= OnInputDeviceChange;
        StopReaders();
    }

    private void OnInputDeviceChange(InputDevice device, InputDeviceChange change)
    {
        if (change == InputDeviceChange.Added && IsSpaceMouseDevice(device))
        {
            Debug.Log($"[SpaceMouse] InputSystem detected device '{device.name}' — removing to release EVIOCGRAB.");
            InputSystem.RemoveDevice(device);
        }
    }

    private void ReleaseFromInputSystem()
    {
        foreach (InputDevice dev in InputSystem.devices.ToArray())
        {
            if (IsSpaceMouseDevice(dev))
            {
                Debug.Log($"[SpaceMouse] Releasing '{dev.name}' from InputSystem to free EVIOCGRAB.");
                InputSystem.RemoveDevice(dev);
            }
        }
    }

    private static bool IsSpaceMouseDevice(InputDevice dev)
    {
        string mfr  = dev.description.manufacturer ?? "";
        string prod = dev.description.product       ?? "";
        return mfr.IndexOf("3Dconnexion", StringComparison.OrdinalIgnoreCase) >= 0
            || prod.IndexOf("SpaceMouse",  StringComparison.OrdinalIgnoreCase) >= 0
            || prod.IndexOf("SpaceNavigator", StringComparison.OrdinalIgnoreCase) >= 0;
    }

    // -------------------------------------------------------------------------
    // Update
    // -------------------------------------------------------------------------

    private void Update()
    {
        if (IsLinux() && Time.unscaledTime >= _nextDiscoveryTime)
        {
            _nextDiscoveryTime = Time.unscaledTime + rediscoveryIntervalSeconds;
            TryExpandReaders();
        }

        if (publishRateHz <= 0f || Time.unscaledTime < _nextPublishTime) return;
        _nextPublishTime = Time.unscaledTime + 1f / publishRateHz;

        if (!TryReadInputs(out Vector3 translation, out Vector3 rotation, out string src))
        {
            LogErrorIfNeeded();
            PublishDriveStopIfNeeded();
            PublishArmStopIfNeeded();
            return;
        }

        LogErrorIfNeeded();

        if (logInput)
            Debug.Log($"[SpaceMouse RAW] src={src}  t=({translation.x:F4},{translation.y:F4},{translation.z:F4})  r=({rotation.x:F4},{rotation.y:F4},{rotation.z:F4})  btn=[{_buttons[0]},{_buttons[1]}]");

        translation = Deadzone(translation, translationDeadzone);
        rotation    = Deadzone(rotation,    rotationDeadzone);

        // Left button rising edge → toggle arm controller via joy2
        if (_buttons[0] == 1 && _prevButtons[0] == 0)
            PublishJoy2Toggle();

        // Update previous button state after edge detection
        _prevButtons[0] = _buttons[0];
        _prevButtons[1] = _buttons[1];

        ControllerManager.ControlMode mode = modeSource != null
            ? modeSource.currentMode
            : ControllerManager.ControlMode.Off;

        switch (mode)
        {
            case ControllerManager.ControlMode.Drive:
                PublishDrive(translation, rotation);
                break;
            case ControllerManager.ControlMode.Arm:
                if (armControl == null || armControl.getArmControlStatus())
                    PublishArm(translation, rotation);
                else
                    PublishArmStopIfNeeded();
                break;
            default:
                PublishDriveStopIfNeeded();
                PublishArmStopIfNeeded();
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Reading
    // -------------------------------------------------------------------------

    private bool TryReadInputs(out Vector3 translation, out Vector3 rotation, out string src)
    {
        translation = Vector3.zero;
        rotation    = Vector3.zero;
        src         = "none";

        EvdevReader reader = SelectReader();
        if (reader == null) return false;

        reader.CopyAxesTo(_axes);
        reader.CopyButtonsTo(_buttons);

        translation = new Vector3(_axes[0], _axes[1], _axes[2]) * translationScale;
        rotation    = new Vector3(_axes[3], _axes[4], _axes[5]) * rotationScale;
        src         = reader.DevicePath;
        _inputError = "";
        return true;
    }

    // -------------------------------------------------------------------------
    // Device discovery
    // -------------------------------------------------------------------------

    private void StartReaders()
    {
        StopReaders();
        TryExpandReaders();
    }

    private void TryExpandReaders()
    {
        foreach (string path in DiscoverPaths())
        {
            bool known = false;
            foreach (EvdevReader r in _readers) if (r.DevicePath == path) { known = true; break; }
            if (known) continue;

            var reader = new EvdevReader(path, reconnectDelayMs);
            _readers.Add(reader);
            reader.Start();
        }

        for (int i = _readers.Count - 1; i >= 0; i--)
            if (!_readers[i].Running && !File.Exists(_readers[i].DevicePath))
                _readers.RemoveAt(i);
    }

    private void StopReaders()
    {
        foreach (EvdevReader r in _readers) r.Stop();
        _readers.Clear();
    }

    private List<string> DiscoverPaths()
    {
        var result = new List<string>();
        AddIfValid(result, explicitEventPath);
        AddByIdPaths(result);
        if (discoverFromProc)   AddProcPaths(result);
        if (discoverEventNodes && result.Count == 0) AddEventNodePaths(result);
        return result;
    }

    private void AddByIdPaths(List<string> paths)
    {
        const string dir = "/dev/input/by-id";
        if (!Directory.Exists(dir)) return;
        try
        {
            string[] files = Directory.GetFiles(dir, "*event*");
            Array.Sort(files);
            foreach (string p in files)
            {
                if (!string.IsNullOrEmpty(byIdNameContains) &&
                    p.IndexOf(byIdNameContains, StringComparison.OrdinalIgnoreCase) < 0) continue;
                AddIfValid(paths, p);
            }
        }
        catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException)
        { _inputError = $"by-id scan failed: {ex.Message}"; }
    }

    private void AddProcPaths(List<string> paths)
    {
        const string proc = "/proc/bus/input/devices";
        if (!File.Exists(proc)) return;
        try
        {
            foreach (string block in File.ReadAllText(proc)
                .Split(new[] { "\n\n" }, StringSplitOptions.RemoveEmptyEntries))
            {
                if (!string.IsNullOrEmpty(procNameContains) &&
                    block.IndexOf(procNameContains, StringComparison.OrdinalIgnoreCase) < 0) continue;

                foreach (string line in block.Split('\n'))
                {
                    if (!line.StartsWith("H:", StringComparison.Ordinal)) continue;
                    int i = line.IndexOf("event", StringComparison.Ordinal);
                    if (i < 0) continue;
                    int e = i + 5;
                    while (e < line.Length && char.IsDigit(line[e])) e++;
                    AddIfValid(paths, "/dev/input/" + line.Substring(i, e - i));
                }
            }
        }
        catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException)
        { _inputError = $"proc scan failed: {ex.Message}"; }
    }

    private void AddEventNodePaths(List<string> paths)
    {
        const string dir = "/dev/input";
        if (!Directory.Exists(dir)) return;
        try
        {
            string[] files = Directory.GetFiles(dir, "event*");
            Array.Sort(files);
            foreach (string p in files) AddIfValid(paths, p);
        }
        catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException)
        { _inputError = $"event scan failed: {ex.Message}"; }
    }

    private void AddIfValid(List<string> paths, string path)
    {
        if (!string.IsNullOrWhiteSpace(path) && !paths.Contains(path) && File.Exists(path))
            paths.Add(path);
    }

    private EvdevReader SelectReader()
    {
        EvdevReader best = null;
        int bestSeq = -1;
        foreach (EvdevReader r in _readers)
        {
            if (!r.Ready) continue;
            if (best == null || r.EventSequence > bestSeq) { best = r; bestSeq = r.EventSequence; }
        }
        if (best == null) _inputError = BuildError();
        return best;
    }

    private string BuildError()
    {
        if (_readers.Count == 0)
            return string.IsNullOrEmpty(_inputError)
                ? "No SpaceMouse event device found under /dev/input/by-id. Is the device plugged in?"
                : _inputError;
        foreach (EvdevReader r in _readers)
            if (!string.IsNullOrEmpty(r.Error)) return r.Error;
        return "SpaceMouse found but no reader is ready. " +
               "Check /dev/input permissions (sudo usermod -aG input $USER then re-login).";
    }

    private void LogErrorIfNeeded()
    {
        string err = string.IsNullOrEmpty(_inputError) ? BuildError() : _inputError;
        if (string.IsNullOrEmpty(err))  { _lastLoggedError = ""; return; }
        if (err == _lastLoggedError) return;
        Debug.LogWarning($"[SpaceMouse] {err}");
        _lastLoggedError = err;
    }

    private static bool IsLinux() =>
        Application.platform == RuntimePlatform.LinuxEditor ||
        Application.platform == RuntimePlatform.LinuxPlayer;

    // -------------------------------------------------------------------------
    // Publishing
    // -------------------------------------------------------------------------

    private void PublishDrive(Vector3 t, Vector3 r)
    {
        if (UdpController.inst == null) return;

        // Kernel reports negative Y when pushing the puck away (forward), so negate by default.
        // invertDriveForward flips that if needed.
        float linearX  = -ReadAxis(driveForwardAxis, t, r) * (invertDriveForward ? -1f : 1f);
        float angularZ =  ReadAxis(driveYawAxis,     t, r) * (invertDriveYaw     ? -1f : 1f);

        float slider = driveSpeedSlider != null ? driveSpeedSlider.value : 1f;
        linearX  *= driveLinearScale  * slider;
        angularZ *= driveAngularScale * slider;

        bool hasInput = Mathf.Abs(linearX) > 0f || Mathf.Abs(angularZ) > 0f;
        if (!hasInput && !_wasDriveActive) return;

        UdpController.inst.PublishMessage(new JObject
        {
            ["topic"]   = "cmd_vel",
            ["msgType"] = "geometry_msgs/msg/Twist",
            ["data"] = new JObject
            {
                ["linear"]  = new JObject { ["x"] = linearX,  ["y"] = 0f, ["z"] = 0f },
                ["angular"] = new JObject { ["x"] = 0f, ["y"] = 0f, ["z"] = angularZ }
            }
        }.ToString());

        _wasDriveActive = hasInput;
    }

    private void PublishArm(Vector3 t, Vector3 r)
    {
        if (UdpController.inst == null) return;

        float slider = armSpeedSlider != null ? armSpeedSlider.value : 1f;
        float scale  = armScale * slider;

        Vector3 lin = Vector3.Scale(t, armLinearSigns)  * scale;
        Vector3 ang = Vector3.Scale(r, armAngularSigns) * scale;

        // SpaceMouse Y (push forward/back) drives twist.linear.z (gripper in/out).
        // Triggers encode it in the gamepad convention expected by joy_to_servo.
        // Inverted: pull back (negative lin.y) → left trigger → positive twist.linear.z (extend).
        float leftTrig  = Mathf.Clamp01(Mathf.Max(0f, -lin.y) * verticalTriggerMultiplier);
        float rightTrig = Mathf.Clamp01(Mathf.Max(0f,  lin.y) * verticalTriggerMultiplier);

        // joy_to_servo IK axis layout (Xbox mapping):
        //   [0] LEFT_STICK_X  → twist.linear.x  = -axes[0]   ← SpaceMouse X  (left/right, negated)
        //   [1] LEFT_STICK_Y  → twist.linear.y  = -axes[1]   ← SpaceMouse Z  (up/down)
        //                                                        lin.z > 0 (up) → axes[1] = +lin.z
        //                                                        → twist.linear.y = -lin.z (gripper -Y = up) ✓
        //   [2] LEFT_TRIGGER  → contributes to twist.linear.z ← SpaceMouse -Y (pull back)
        //   [3] RIGHT_STICK_X → twist.angular.y = -axes[3]   ← SpaceMouse RZ (yaw/twist)
        //   [4] RIGHT_STICK_Y → twist.angular.x =  axes[4]   ← SpaceMouse RX (pitch)
        //   [5] RIGHT_TRIGGER → contributes to twist.linear.z ← SpaceMouse +Y (push forward)
        //   [6] D_PAD_X       → twist.angular.z =  axes[6]   ← SpaceMouse RY (roll + ε)
        //   [7] D_PAD_Y       → shoulder joint (joint mode, unmapped)
        float[] axes = {
             lin.x,      // X inverted: negate so right → positive twist.linear.x
            -lin.z,      // lifting cap (positive lin.z) → axes[1] = -lin.z
                         // → twist.linear.y = -(-lin.z) = +lin.z → gripper moves up ✓
            leftTrig,
            -ang.z,  ang.x,   // yaw ← RZ (twist), pitch ← RX
            rightTrig,
            ang.y + wristRollZeroEpsilon,  // roll ← RY
            0f
        };

        int[] btns = new int[Mathf.Max(joyButtonCount, 1)];
        SetBtn(btns, smButton0JoyIndex, _buttons[0]);
        SetBtn(btns, smButton1JoyIndex, _buttons[1]);

        bool hasInput = lin != Vector3.zero || ang != Vector3.zero
                     || _buttons[0] != 0 || _buttons[1] != 0;
        if (!hasInput && !_wasArmActive) return;

        UdpController.inst.PublishMessage(new JObject
        {
            ["topic"]   = "joy",
            ["msgType"] = "sensor_msgs/msg/Joy",
            ["data"] = new JObject
            {
                ["header"]  = new JObject
                {
                    ["stamp"]    = new JObject { ["sec"] = 0, ["nanosec"] = 0 },
                    ["frame_id"] = ""
                },
                ["axes"]    = new JArray(axes),
                ["buttons"] = new JArray(btns)
            }
        }.ToString());

        _wasArmActive = hasInput;
    }

    private void SetBtn(int[] arr, int idx, int val)
    { if (idx >= 0 && idx < arr.Length) arr[idx] = val; }

    private void PublishDriveStopIfNeeded() { if (_wasDriveActive) PublishDrive(Vector3.zero, Vector3.zero); }
    private void PublishArmStopIfNeeded()   { if (_wasArmActive)   PublishArm  (Vector3.zero, Vector3.zero); }

    // Toggles the arm IK controller on/off by sending a joy2 message with button[joy2ButtonIndex].
    // Mirrors the workflow in Drive.cs (joystickEast handler) but driven by the SpaceMouse left button.
    // Each press flips the state: engaged → released → engaged → …
    private void PublishJoy2Toggle()
    {
        if (UdpController.inst == null) return;

        _joy2Active = !_joy2Active;

        int[] btns = new int[Mathf.Max(joy2ButtonCount, joy2ButtonIndex + 1)];
        btns[joy2ButtonIndex] = _joy2Active ? 1 : 0;

        JObject msg = new JObject
        {
            ["topic"]   = "joy2",
            ["msgType"] = "sensor_msgs/msg/Joy",
            ["data"] = new JObject
            {
                ["header"]  = new JObject
                {
                    ["stamp"]    = new JObject { ["sec"] = 0, ["nanosec"] = 0 },
                    ["frame_id"] = ""
                },
                ["axes"]    = new JArray(new float[] { 0f }),
                ["buttons"] = new JArray(btns)
            }
        };

        UdpController.inst.PublishMessage(msg.ToString());
        Debug.Log($"[SpaceMouse] joy2 toggle → button[{joy2ButtonIndex}] = {btns[joy2ButtonIndex]}  (_joy2Active={_joy2Active})");
    }

    private float ReadAxis(AxisChoice a, Vector3 t, Vector3 r)
    {
        switch (a)
        {
            case AxisChoice.TranslationX: return t.x;
            case AxisChoice.TranslationY: return t.y;
            case AxisChoice.TranslationZ: return t.z;
            case AxisChoice.RotationX:    return r.x;
            case AxisChoice.RotationY:    return r.y;
            case AxisChoice.RotationZ:    return r.z;
            default: return 0f;
        }
    }

    private Vector3 Deadzone(Vector3 v, float threshold) =>
        new Vector3(Deadzone(v.x, threshold), Deadzone(v.y, threshold), Deadzone(v.z, threshold));

    private float Deadzone(float v, float threshold) => Mathf.Abs(v) < threshold ? 0f : v;

    // =========================================================================
    // EvdevReader — background thread that reads raw Linux evdev events
    // =========================================================================

    private class EvdevReader
    {
        public readonly string DevicePath;

        private readonly object  _lock          = new object();
        private readonly float[] _absAxes       = new float[AxCount];
        private readonly float[] _relAxes       = new float[AxCount];
        private readonly bool[]  _seenAbs       = new bool[AxCount];
        private readonly int[]   _btnStates     = new int[BtnCount];
        private readonly int     _reconnDelayMs;

        private Thread     _thread;
        private FileStream _stream;
        private volatile bool _running  = false;
        private volatile bool _ready    = false;
        private volatile int  _evSeq    = 0;
        private string _error = "";

        public bool   Ready         => _ready;
        public bool   Running       => _running;
        public int    EventSequence => _evSeq;
        public string Error         => _error;

        public EvdevReader(string path, int reconnectDelayMs)
        {
            DevicePath     = path;
            _reconnDelayMs = reconnectDelayMs;
        }

        public void Start()
        {
            if (_running) return;
            _running = true;
            _thread  = new Thread(ReadLoop) { IsBackground = true, Name = $"SpaceMouse {DevicePath}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            try { _stream?.Close(); } catch { }
            _stream = null;
        }

        public void CopyAxesTo(float[] target)
        {
            lock (_lock)
            {
                for (int i = 0; i < Math.Min(target.Length, _absAxes.Length); i++)
                {
                    target[i]   = _seenAbs[i] ? _absAxes[i] : _relAxes[i];
                    _relAxes[i] = 0f;
                }
            }
        }

        public void CopyButtonsTo(int[] target)
        {
            lock (_lock)
                for (int i = 0; i < Math.Min(target.Length, _btnStates.Length); i++)
                    target[i] = _btnStates[i];
        }

        // Outer loop retries after disconnect so a replugged device is re-acquired.
        private void ReadLoop()
        {
            byte[] buf = new byte[64];

            while (_running)
            {
                try
                {
                    using (var fs = new FileStream(DevicePath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                    {
                        _stream = fs;
                        _error  = "";
                        _ready  = true;

                        while (_running)
                        {
                            int n = fs.Read(buf, 0, buf.Length);
                            if (n <= 0) continue;

                            int evSize = SelectEventSize(buf, n);
                            if (evSize <= 0) continue;

                            int tOff = evSize == 24 ? 16 : 8;

                            for (int off = 0; off + evSize <= n; off += evSize)
                            {
                                ushort type = BitConverter.ToUInt16(buf, off + tOff);
                                ushort code = BitConverter.ToUInt16(buf, off + tOff + 2);
                                int    val  = BitConverter.ToInt32 (buf, off + tOff + 4);

                                lock (_lock)
                                {
                                    if (type == EvTypeAbs || type == EvTypeRel)
                                    {
                                        int ai = AxisIndex(code);
                                        if (ai >= 0)
                                        {
                                            if (type == EvTypeAbs) { _absAxes[ai] = val; _seenAbs[ai] = true; }
                                            else                   { _relAxes[ai] += val; }
                                        }
                                    }
                                    else if (type == EvTypeKey)
                                    {
                                        int bi = (int)code - BtnBase;
                                        if (bi >= 0 && bi < BtnCount)
                                            _btnStates[bi] = val != 0 ? 1 : 0;
                                    }
                                }
                                _evSeq++;
                            }
                        }
                    }
                }
                catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException || ex is ObjectDisposedException)
                {
                    _ready  = false;
                    _stream = null;
                    _error  = $"SpaceMouse lost on {DevicePath}: {ex.Message}. Retrying in {_reconnDelayMs} ms…";
                    if (_running) Thread.Sleep(_reconnDelayMs);
                }
            }

            _ready = false;
        }

        private int SelectEventSize(byte[] buf, int n)
        {
            int s24 = Score(buf, n, 24, 16), s16 = Score(buf, n, 16, 8);
            if (s24 == 0 && s16 == 0)                      return -1;
            if (s24 > s16)                                  return 24;
            if (s16 > s24)                                  return 16;
            if (n % 24 == 0 && n % 16 != 0)                return 24;
            if (n % 16 == 0 && n % 24 != 0)                return 16;
            return 24;
        }

        private int Score(byte[] buf, int n, int evSize, int tOff)
        {
            if (n < evSize) return 0;
            int score = 0;
            for (int i = 0; i < Math.Min(n / evSize, 4); i++)
            {
                ushort t = BitConverter.ToUInt16(buf, i * evSize + tOff);
                if (t != EvTypeAbs && t != EvTypeRel && t != EvTypeKey) continue;
                score += 2;
                ushort c = BitConverter.ToUInt16(buf, i * evSize + tOff + 2);
                if (AxisIndex(c) >= 0) score++;
            }
            return score;
        }

        private static int AxisIndex(ushort code)
        {
            switch (code)
            {
                case AxisX:  return 0;
                case AxisY:  return 1;
                case AxisZ:  return 2;
                case AxisRx: return 3;
                case AxisRy: return 4;
                case AxisRz: return 5;
                default:     return -1;
            }
        }
    }
}
