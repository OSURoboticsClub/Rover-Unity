using UnityEngine;

public class AutonomyObjectDetectionUI : MonoBehaviour
{
    public GameObject objectDetectionPanel;
    
    private bool isOpen = false;

    public void ToggleObjectDetectionPanel()
    {
        Debug.Log("Opening Object Detection Panel");

        isOpen = !isOpen;
        objectDetectionPanel.SetActive(isOpen);
    }

    public void ClosePanel()
    {
        Debug.Log("Closing Object Detection Panel");

        isOpen = false;
        objectDetectionPanel.SetActive(false);
    }
}