using System.Collections;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

public class publishJointAngles : MonoBehaviour
{
    RobotArmController robotArm;
    KeyboardController keyboardController;

    [SerializeField] private TextAsset joy2MessageJson;

    [SerializeField] private TextAsset floatArrMessageJson;

    Coroutine currentCoroutine = null;

    bool allowArmControl = true;

    List<float> preset_pose_0 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> preset_pose_1 = new List<float> { 0.0f, -0.34f, -1.98968f, 0.0f, 0.785398f, 0.0f };
    List<float> preset_pose_2 = new List<float> { 0.0f, -0.698132f, -1.65806f, 0.0f, -0.785698f, 0.0f };
    List<float> preset_pose_3 = new List<float> { 0.17453f, 1.22173f, -2.61799f, 0.0f, -0.17453f, 0.0f };
    List<float> preset_pose_4 = new List<float> { 0.0382f, -0.9648478f, -2.1777784f,-0.01376f, 1.557087150f, 0.016154f };
    List<float> preset_pose_5 = new List<float> { 0.0382f, -0.9648478f, -2.1777784f, -0.01376f,1.557087150f, -3.1415926f };

    List<float> science_pose_0 = new List<float> { -1.5708f, 0.97738f, 2.44346f, -0.17453f, 0.0f, 0.0f };
    List<float> science_pose_1 = new List<float> { 1.5708f, 0.92502f, -1.8326f, 0.40143f, 1.5708f, 0.0f };
    List<float> science_pose_2 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> science_pose_3 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> science_pose_4 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    List<float> science_pose_5 = new List<float> { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };


    void Awake()
    {
        robotArm = FindObjectOfType<RobotArmController>();
        keyboardController = FindObjectOfType<KeyboardController>();
    }

    public void Publish(List<float> angles)
    {

        string msg = "set_joint_angles";
        for (int i = 0; i < angles.Count; i++)
        {
            msg += ";" + angles[i];
        }

        robotArm.visualize_goal(angles);

        // Stop previous coroutine if still waiting
        if (currentCoroutine != null)
        {
            StopCoroutine(currentCoroutine);
        }

        currentCoroutine = StartCoroutine(SendAngles(angles));
    }

    public void publish_preset_pose_0()
    {
        Debug.Log(currentCoroutine);
        Publish(preset_pose_0);
    }
    public void publish_preset_pose_1()
    {
        Debug.Log(currentCoroutine);

        Publish(preset_pose_1);
    }
    public void publish_preset_pose_2()
    {
        Debug.Log(currentCoroutine);

        Publish(preset_pose_2);
    }
    public void publish_preset_pose_3()
    {
        Debug.Log(currentCoroutine);
        Publish(preset_pose_3);
    }
    public void publish_preset_pose_4()
    {
        Debug.Log(currentCoroutine);
        Publish(preset_pose_4);
    }
    public void publish_preset_pose_5()
    {
        Debug.Log(currentCoroutine);
        Publish(preset_pose_5);
    }

    public void publish_science_pose_0()
    {
        Debug.Log(currentCoroutine);
        Publish(science_pose_0);
    }

    public void publish_science_pose_1()
    {
        Debug.Log(currentCoroutine);
        Publish(science_pose_1);
    }

    public void publish_science_pose_2()
    {
        Debug.Log(currentCoroutine);
        Publish(science_pose_2);
    }

    public void publish_science_pose_3()
    {
        Debug.Log(currentCoroutine);
        Publish(science_pose_3);
    }

    public void publish_science_pose_4()
    {
        Debug.Log(currentCoroutine);
        Publish(science_pose_4);
    }

    public void publish_science_pose_5()
    {
        Debug.Log(currentCoroutine);
        Publish(science_pose_5);
    }

    public void publish_custom_pose(List<float> customPose)
    {
        Publish(customPose);
    }

    public void nullSendAnglesProcess()
    {
        StopCoroutine(currentCoroutine);
        currentCoroutine = null;

    }
    public bool getArmControlStatus()
    {
        return allowArmControl;
    }

    IEnumerator SendAngles(List<float> pose_angles)
    {
        // Wait for confirmation
        yield return new WaitUntil(() => keyboardController.sendArmCommand);
        allowArmControl = false;


        // JObject joy2_json = JObject.Parse(joy2MessageJson.text);
        // joy2_json["topic"] = "joy2";
        // joy2_json["data"]["axes"] = new JArray(new float[] {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f});
        // joy2_json["data"]["buttons"] = new JArray(new int[] {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0});
        // string joy2_msg = joy2_json.ToString();

        // UdpController.inst.PublishMessage(joy2_msg);
        // yield return new WaitForSeconds(0.15f);

        JObject floatArrMessage = JObject.Parse(floatArrMessageJson.text);
        floatArrMessage["topic"] = "set_joint_angles";
        floatArrMessage["data"]["data"]= new JArray(pose_angles.ToArray());

        string msg = floatArrMessage.ToString();
        UdpController.inst.PublishMessage(msg);

        yield return new WaitForSeconds(7f);
        // joy2_json["data"]["buttons"] = new JArray(new int[] {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});
        // joy2_msg = joy2_json.ToString();
        // UdpController.inst.PublishMessage(joy2_msg);
        allowArmControl = true;
        currentCoroutine = null; // Clear coroutine reference
        robotArm.remove_vis();
    }
}
