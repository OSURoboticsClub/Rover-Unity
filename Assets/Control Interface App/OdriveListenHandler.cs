using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using TMPro;

public class OdriveListenHandler : MonoBehaviour
{
    public static OdriveListenHandler inst;
    public GameObject[] driveNodes; 
    public GameObject[] armNodes;
    public GameObject[] driveText;
    public GameObject[] armText;
    


    void Awake()
    {
        inst = this;
    }

    public void ReceiveOdriveTelem(string message)
    {
        Debug.Log("Received /nodetopiclisten message: " + message);
        string[] splitStatusMessage = message.Split('?');
        string arbitration = splitStatusMessage[0];
        string motorId = splitStatusMessage[1];
        string temp = splitStatusMessage[2];
        string disarm_reason = splitStatusMessage[3];

        string[] splitTopicCanNet = arbitration.Split(';');

        string busName = splitTopicCanNet[1];

        if(busName=="can0"){
            ColorNetwork(disarm_reason,motorId,temp,driveNodes,driveText);
        }
        if(busName=="can1"){
            ColorNetwork(disarm_reason,motorId,temp,armNodes,armText);
        }
       
        
       
        
    }
    public void ColorNetwork(string disarm_reason, string motorId, string temp,GameObject[] nodes, GameObject[] textArray)
    {
        
        string[] reasons = disarm_reason.Split(";");
        string[] idList = motorId.Split(';');
        string[] tempList = temp.Split(';');
        foreach (string str_id in idList)
        {
            
            
            int int_id = -1;
            int.TryParse(str_id, out int_id);
            string reason = reasons[int_id];
            string display_temp = tempList[int_id];

            Color node_color = Color.red;
            if(reason == "NO ERROR")
            {
                node_color = Color.green;
            } 

            string network_name = "0."+str_id;
            if(nodes == armNodes)
            {
                network_name = "1."+str_id;
            }

            foreach(GameObject node in nodes)
            {
                string node_name = node.name;
                RawImage img = node.GetComponent<RawImage>();
                if(network_name == node_name)
                {
                    img.color = node_color;
                }

            }
            string text_name = network_name + "t";
            foreach(GameObject text in textArray)
            {
                string node_name = text.name;
                TextMeshProUGUI msg = text.GetComponent<TextMeshProUGUI>();

                if(text_name == node_name && reason == "NO ERROR")
                {
                    msg.text = display_temp;
                }

            }
            



        }
        
    }
}