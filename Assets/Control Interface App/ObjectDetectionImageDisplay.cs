using UnityEngine;
using UnityEngine.UI;

public class ObjectDetectionImageDisplay : MonoBehaviour
{
    [SerializeField] RawImage display;
    [SerializeField] AspectRatioFitter aspectRatioFitter;
    
    private Texture2D texture;
    private bool isActive;

    public void Open()
    {
        Debug.Log("Opening Object Display");
        isActive = true;
        display.enabled = true;
    }

    public void Close()
    {
        Debug.Log("Closing Object Display");

        isActive = false;
        display.enabled = false;
    }

    public void UpdateImage(byte[] imageData, int width, int height, TextureFormat format) 
    {
        if (!isActive) return;
        if (texture == null || texture.width != width || texture.height != height) 
        {
            texture = new Texture2D(width, height, format, false);
            display.texture = texture;

            aspectRatioFitter.aspectRatio = (float)width/height;
        }

        texture.LoadRawTextureData(imageData);
        texture.Apply();
    }
}