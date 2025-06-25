using UnityEngine;

public class CanvasController : MonoBehaviour
{
    public GameObject MainCanvas;
    public GameObject CameraCanvas;
    public GameObject StatusCanvas;

    public void SwitchToMainCanvas()
    {
        Debug.Log("Switching to Main Canvas");

        MainCanvas.SetActive(true);
        CameraCanvas.SetActive(false);
        StatusCanvas.SetActive(false);
    }

    public void SwitchToCameraCanvas()
    {
        Debug.Log("Switching to Camera Canvas");

        MainCanvas.SetActive(false);
        CameraCanvas.SetActive(true);
        StatusCanvas.SetActive(false);
    }
    public void SwitchToStatusCanvas()
    {
        Debug.Log("Switching to Status Canvas");

        MainCanvas.SetActive(false);
        CameraCanvas.SetActive(false);
        StatusCanvas.SetActive(true);
    }
}

