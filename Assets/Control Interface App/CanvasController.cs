using UnityEngine;

public class CanvasController : MonoBehaviour
{
    public GameObject MainCanvas;
    public GameObject CameraCanvas;

    public void SwitchToMainCanvas()
    {
        Debug.Log("Switching to Main Canvas");

        MainCanvas.SetActive(true);
        CameraCanvas.SetActive(false);
    }

    public void SwitchToCameraCanvas()
    {
        Debug.Log("Switching to Camera Canvas");

        MainCanvas.SetActive(false);
        CameraCanvas.SetActive(true);
    }
}

