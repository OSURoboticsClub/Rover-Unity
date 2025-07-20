using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

// if your mouse is over the map, this returns the world coordinates of the mouse location
// this is complciated because the image in the UI that contains the map is basically a projection of
// ANOTHER camera. So a bit of transforming has to take place.

public class GetMapMousePosition : MonoBehaviour
{
    public static GetMapMousePosition inst;
    public Camera targetCamera;       // The camera rendering to the RenderTexture
    public Camera secondCam;
    public RawImage rawImage;         // The RawImage showing the RenderTexture

    private void Awake() {
        inst = this;
    }

    public Vector3? GetWorldPositionFromMouse() {
        if (targetCamera == null || rawImage == null)
            return null;

        RectTransform rectTransform = rawImage.rectTransform;
        Vector2 localPoint;

        // Convert screen point to local UI coordinates
        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(
            rectTransform,
            Input.mousePosition,
            secondCam,  // null if Canvas is Overlay; provide camera if Screen Space - Camera
            out localPoint))
            return null;

        // Normalize to UV space (0 to 1)
        Rect rect = rectTransform.rect;
        float uvX = (localPoint.x - rect.x) / rect.width;
        float uvY = (localPoint.y - rect.y) / rect.height;

        // Check if mouse is outside the image bounds
        if (uvX < 0 || uvX > 1 || uvY < 0 || uvY > 1)
            return null;

        // Convert to world position using ViewportToWorldPoint
        Vector3 viewportPoint = new Vector3(uvX, uvY, targetCamera.nearClipPlane);
        return targetCamera.ViewportToWorldPoint(viewportPoint);
    }
}
