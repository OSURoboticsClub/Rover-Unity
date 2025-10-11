using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class CameraControl : MonoBehaviour, IPointerDownHandler, IPointerUpHandler, IDragHandler
{
    public static CameraControl inst;
    public Camera secondCamera; // Assign the second camera in the Inspector
    [SerializeField] Transform roverIcon;
    [SerializeField] Transform destIcon;
    [SerializeField] Transform iconsParent;
    [SerializeField] Transform camCornerGameobject;
    private Vector3 initialMouseWorldPosition;
    private Vector3 initialSecondCameraPosition;
    private bool isDragging = false;
    [SerializeField] Vector2 delta;
    [SerializeField] float scaleX;
    [SerializeField] float scaleY;
    public float zoomSpeed = 1f; // Speed of zooming
    public float minOrthoSize = 1f; // Minimum orthographic size
    public float maxOrthoSize = 20f; // Maximum orthographic size

    [SerializeField] RawImage map;
    [SerializeField] Vector2 worldPosOfMouseDown;
    [SerializeField] Vector2 offset;
    [SerializeField] float iconScale = 1f;
    public float lineScale = 1f;
    public float iconScaleForZoom = 1f;
    public float mapImageScaleFactor = 1f;
    [SerializeField] float draggingMultiplier;

    void Awake()
    {
        inst = this;
        //mapImageScaleFactor = Screen.height / map.rectTransform.sizeDelta.y;
        iconScaleForZoom = 2f * secondCamera.orthographicSize;
    }


    void Update()
    {
        // Get the scroll input
        float scrollInput = Input.GetAxis("Mouse ScrollWheel");
        //if (scrollInput == 0f) return; // return if the user didn't scroll the mouse wheel since last frame

        
        Vector2 oldMousePos = GetWorldPositionOfMouse(Input.mousePosition);
        Vector2 oldMouseScreenPos = Input.mousePosition;
        secondCamera.orthographicSize -= scrollInput * zoomSpeed * secondCamera.orthographicSize;
        secondCamera.orthographicSize = Mathf.Clamp(secondCamera.orthographicSize, minOrthoSize, maxOrthoSize);

        Vector2 newMousepos = GetWorldPositionOfMouse(oldMouseScreenPos);
        Vector3 diff = newMousepos - oldMousePos;
        secondCamera.transform.position -= diff * mapImageScaleFactor;

        if (scrollInput != 0f) RescaleIcons();

        float height = 2f * secondCamera.orthographicSize;
        MapController.instance.SetLineScale(height * lineScale);
    }

    public void RescaleIcons()
    {
        iconScaleForZoom = 2f * secondCamera.orthographicSize;
        foreach (Transform child in iconsParent)
        {
            child.localScale = iconScaleForZoom * iconScale * Vector3.one;
        }
    }

    Vector2 GetWorldPositionOfMouse(Vector3 mouseScreenPosition)
    {
        // Get the world position of the RawImage center
        RectTransform rawImageRectTransform = map.rectTransform;
        Vector3 rawImageCenterScreenPosition = RectTransformUtility.WorldToScreenPoint(Camera.main, rawImageRectTransform.position);

        // Calculate the offset
        offset = mouseScreenPosition - rawImageCenterScreenPosition;
        offset = new Vector2(offset.x / map.rectTransform.sizeDelta.x, offset.y / map.rectTransform.sizeDelta.y);
        float cameraHeight = secondCamera.orthographicSize * 2;
        float cameraWidth = cameraHeight * secondCamera.aspect;
        Vector2 world = new Vector2(secondCamera.transform.position.x + offset.x * cameraWidth, secondCamera.transform.position.y + offset.y * cameraHeight);
        return world;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left && IsMouseOverMap())
        {
            isDragging = true;

            // Store the initial mouse screen position when the middle mouse button is pressed
            initialMouseWorldPosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
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
            Vector3 worldDelta = Camera.main.ScreenToWorldPoint(Input.mousePosition) - initialMouseWorldPosition;
            float proportion = secondCamera.orthographicSize / Camera.main.orthographicSize;
            worldDelta = new Vector2(worldDelta.x, worldDelta.y) * proportion * draggingMultiplier * 2;
            delta = worldDelta;
            secondCamera.transform.position = initialSecondCameraPosition - worldDelta;
        }
    }

    public bool IsMouseOverMap()
    {
        RawImage image = map;
        if (image == null) return false;

        PointerEventData pointerData = new PointerEventData(EventSystem.current) {
            position = Input.mousePosition
        };

        List<RaycastResult> results = new List<RaycastResult>();
        EventSystem.current.RaycastAll(pointerData, results);

        if (results.Count == 0) return false;

        // The topmost element is results[0]
        return results[0].gameObject == map.gameObject;
    }
}
