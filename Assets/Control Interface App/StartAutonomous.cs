using UnityEngine;

namespace ROS2
{

/// <summary>
/// An example class provided for testing of basic ROS2 communication
/// </summary>
public class StartAutonomous : MonoBehaviour
{
    // Start is called before the first frame update
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.String> chatter_pub;
    private int i;
    public static StartAutonomous instance;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        instance = this;
    }

public void Send(GpsLocation coordinateData){
	if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("UnityAutonomousControlNode");
                chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("AutonomousControl");
            }

            i++;
            std_msgs.msg.String msg = new std_msgs.msg.String();
            string latAndLong = "(" + coordinateData.lat.text + ", " + coordinateData.lon.text + ")";
            msg.Data = "Go to " + latAndLong;
            chatter_pub.Publish(msg);
            Debug.Log(msg.Data);
        }
}
}

}  // namespace ROS2
