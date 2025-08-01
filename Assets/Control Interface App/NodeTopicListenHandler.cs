using UnityEngine;
using UnityEngine.UI;

public class NodeTopicListenHandler : MonoBehaviour
{
    public static NodeTopicListenHandler inst;
    public GameObject[] myObjects; 
    public GameObject[] nodeObjects;
   

    void Awake()
    {
        inst = this;
    }

    public void ReceiveNodeTopicListen(string message)
    {
//        Debug.Log("Received /nodetopiclisten message: " + message);
        string[] splitTopicsNodes = message.Split('?');
        decodeNodes(splitTopicsNodes[1]);
        decodeTopics(splitTopicsNodes[0]);
       
        
    }
    private void decodeTopics(string message)
    {
    // Split the message into topic-value pairs
        string[] splitTopics = message.Split(';');
        
        // Iterate through GameObjects and try to match with topic names
        foreach (GameObject topicObject in myObjects)
        {
            
            string objectName = topicObject.name;
            RawImage img = topicObject.GetComponent<RawImage>();
            foreach (string topicStatusPair in splitTopics)
            {

                // Skip empty parts (especially if message ends in ;)
                if (string.IsNullOrWhiteSpace(topicStatusPair))
                    continue;

                // Split into name and value by ','
                string[] splitNameStatus = topicStatusPair.Split(',');

                if (splitNameStatus.Length != 2)
                {
                    //Debug.LogWarning("Malformed topicStatusPair: " + topicStatusPair);
                    continue;
                }

                string topicName = splitNameStatus[0].Trim();
                string status = splitNameStatus[1].Trim();

                if (topicName == objectName && status == "A")
                {
                    
                    img.color = Color.green;  // Set the UI element to green
                    //Debug.Log($"Matched {topicName} to object {objectName} with status {status}");
                    // You can now do something with the GameObject, e.g., color it based on status
                    break;
                } else {
                    img.color = Color.red;  // Set the UI element to green
                
                }
            }
        }
    }
    
    private void decodeNodes(string message)
    {
    // Split the message into node-value pairs
        string[] splitNodes = message.Split(';');
        
        // Iterate through GameObjects and try to match with topic names
        foreach (GameObject nodeObject in nodeObjects)
        {
            
            string objectName = nodeObject.name;
            RawImage img = nodeObject.GetComponent<RawImage>();
            foreach (string nodeStatusPair in splitNodes)
            {

                // Skip empty parts (especially if message ends in ;)
                if (string.IsNullOrWhiteSpace(nodeStatusPair))
                    continue;

                // Split into name and value by ','
                string[] splitNameStatus = nodeStatusPair.Split(',');

                if (splitNameStatus.Length != 2)
                {
                    //Debug.LogWarning("Malformed topicStatusPair: " + nodeStatusPair);
                    continue;
                }

                string nodeName = splitNameStatus[0].Trim();
                string status = splitNameStatus[1].Trim();
                
                if (nodeName == objectName && status == "A")
                {
                    
                    img.color = Color.green;  // Set the UI element to green
                    //Debug.Log($"Matched {nodeName} to object {objectName} with status {status}");
                    // You can now do something with the GameObject, e.g., color it based on status
                    break;
                } else {
                    img.color = Color.red;  // Set the UI element to green
                
                }
            }
        }
    }
}

