using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class CameraControl : MonoBehaviour, IPointerDownHandler, IPointerUpHandler, IDragHandler
{
    public Camera secondCamera; // Assign the second camera in the Inspector
    private Vector3 initialMouseScreenPosition;
    private Vector3 initialSecondCameraPosition;
    private bool isDragging = false;
    [SerializeField] Vector2 delta;
    [SerializeField] float scaleX;
    [SerializeField] float scaleY;
    public float zoomSpeed = 1f; // Speed of zooming
    public float minOrthoSize = 1f; // Minimum orthographic size
    public float maxOrthoSize = 20f; // Maximum orthographic size

    [SerializeField] RawImage img;
    [SerializeField] Vector2 worldPosOfMouseDown;
    [SerializeField] Vector2 offset;
    [SerializeField] Transform roverIcon;
    [SerializeField] Transform destIcon;
    [SerializeField] float iconScale = 1f;


    void Update()
    {
        // Get the scroll input
        float scrollInput = Input.GetAxis("Mouse ScrollWheel");

        // Adjust the orthographic size based on scroll input
        if (scrollInput != 0f)
        {
            Vector2 oldMousePos = GetWorldPositionOfMouse(Input.mousePosition);
            Vector2 oldMouseScreenPos = Input.mousePosition;
            secondCamera.orthographicSize -= scrollInput * zoomSpeed * secondCamera.orthographicSize;

            // Clamp the orthographic size to stay within limits
            secondCamera.orthographicSize = Mathf.Clamp(secondCamera.orthographicSize, minOrthoSize, maxOrthoSize);
            Vector2 newMousepos = GetWorldPositionOfMouse(oldMouseScreenPos);
            Vector3 diff = newMousepos - oldMousePos;
            secondCamera.transform.position -= diff;

            float height = 2f * secondCamera.orthographicSize;
            roverIcon.transform.localScale = height * iconScale * Vector3.one;
            destIcon.transform.localScale = height * iconScale * Vector3.one;
        }
    }

    Vector2 GetWorldPositionOfMouse(Vector3 mouseScreenPosition)
    {
        // Get the world position of the RawImage center
        RectTransform rawImageRectTransform = img.rectTransform;
        Vector3 rawImageCenterScreenPosition = RectTransformUtility.WorldToScreenPoint(Camera.main, rawImageRectTransform.position);

        // Calculate the offset
        offset = mouseScreenPosition - rawImageCenterScreenPosition;
        offset = new Vector2(offset.x / img.rectTransform.sizeDelta.x, offset.y / img.rectTransform.sizeDelta.y);
        float cameraHeight = secondCamera.orthographicSize * 2;
        float cameraWidth = cameraHeight * secondCamera.aspect;
        Vector2 world = new Vector2(secondCamera.transform.position.x + offset.x * cameraWidth, secondCamera.transform.position.y + offset.y * cameraHeight);
        return world;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left)
        {
            isDragging = true;

            // Store the initial mouse screen position when the middle mouse button is pressed
            initialMouseScreenPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            initialSecondCameraPosition = secondCamera.transform.position;
        }
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left)
        {
            isDragging = false;
        }
    }

    public void OnDrag(PointerEventData eventData)
    {
        if (isDragging)
        {
            Vector3 worldDelta = Camera.main.ScreenToWorldPoint(Input.mousePosition) - initialMouseScreenPosition;
            float porportion = secondCamera.orthographicSize / Camera.main.orthographicSize;
            worldDelta = new Vector2(worldDelta.x * scaleX, worldDelta.y * scaleY) * porportion;
            delta = worldDelta;
            secondCamera.transform.position = initialSecondCameraPosition - worldDelta;
        }
    }
}