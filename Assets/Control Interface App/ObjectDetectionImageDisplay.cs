using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class ObjectDetectionImageDisplay : MonoBehaviour
{
    public static ObjectDetectionImageDisplay Instance;

    [SerializeField] private RawImage display;
    [SerializeField] private RectTransform boundingBox;
    [SerializeField] private TMP_Text label;
    [SerializeField] AspectRatioFitter aspectRatioFitter;

    public void Awake()
    {
        Instance = this;
    }

    public void DisplayDetection(
        Texture2D frame,
        Vector2 topLeft,
        Vector2 bottomRight,
        float confidence,
        string objectName) 
    {
        display.texture = frame;
        DrawBoundingBox(frame.width, frame.height, topLeft, bottomRight);
        label.text = $"{objectName} ({confidence * 100f:F1})";
    }

    void DrawBoundingBox(int width, int height, Vector2 tl, Vector2 br) 
    {
        float x = tl.x;
        float y = tl.y;
        float boxWidth = br.x - tl.x;
        float boxHeight = br.y - tl.y;
        // convert pixel space to UI space
        float uiX = x - width / 2f;
        float uiY = height / 2f - y;
        
        boundingBox.anchoredPosition = new Vector2 (
            uiX + boxWidth / 2f,
            uiY - boxHeight / 2f
        );
        
        boundingBox.sizeDelta = new Vector2(boxWidth, boxHeight);
    }
}