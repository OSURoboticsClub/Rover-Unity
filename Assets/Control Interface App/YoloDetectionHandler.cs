using UnityEngine;
using UnityEngine.UI;

public class YoloDetectionHandler : MonoBehaviour
{
    public static YoloDetectionHandler Instance;

    public void Awake()
    {
        Instance = this;
    }

    public void HandleDetection(YoloDetectionMsg msg) 
    {
        if (!msg.detected) return;

        Texture2D frame = CameraStreamManager.Instance.GetFrame(msg.camera_name);
        if (frame == null) return;

        ObjectDetectionImageDisplay.Instance.DisplayDetection(
            // information retrieved from ROS topic msg
            frame,
            msg.top_left,
            msg.bottom_right,
            msg.confidence,
            GetObjectName(msg.object_type)
        );
    }

    void GetObjectName(int id) 
    {
        return id switch 
        {
            1 => "Mallet",
            2 => "Hammer"
            3 => "Waterbottle"
            4 => "ArUco",
            _ => "Unknown"
        };
    }
}