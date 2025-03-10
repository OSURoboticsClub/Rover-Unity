using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI; // Needed for RawImage

public class ArmCameraOrbit : MonoBehaviour
{
    public Vector3 target; // The target vector to orbit and pan around

    [SerializeField] Camera cam; // Camera component
    [SerializeField] Transform arm;
    [SerializeField] RawImage img;
    [SerializeField] Canvas canvas;
    public float xSpeed = 120.0f;
    public float ySpeed = 120.0f;

    public float cameraPanModifier = 0.1f;

    public float yMinLimit = -20f; // Minimum vertical angle
    public float yMaxLimit = 80f; // Maximum vertical angle

    public float sizeMin = 1f; // Minimum orthographic size
    public float sizeMax = 20f; // Maximum orthographic size

    float x = 0.0f;
    float y = 0.0f;
    [SerializeField] float zoomSpeed = 1f;

    private bool isDragging = false; // Track if the user started dragging inside the image
    private Vector3 lastMousePosition; // Store last known mouse position

    void Start()
    {
        Vector3 angles = cam.transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        target = arm.position;
    }

    void LateUpdate()
    {
        // Detect mouse click over image
        if ((Input.GetMouseButtonDown(1) || Input.GetMouseButtonDown(0)) && IsMouseOverImage())
        {
            isDragging = true;
            lastMousePosition = Input.mousePosition; // Store starting mouse position
        }

        // Stop dragging when the mouse button is released
        if (Input.GetMouseButtonUp(1) || Input.GetMouseButtonUp(0))
        {
            isDragging = false;
        }

        // Only allow rotation if dragging was initiated on the image
        if (isDragging && Input.GetMouseButton(1))
        {
            Vector3 currentMousePosition = Input.mousePosition;
            Vector3 delta = currentMousePosition - lastMousePosition;

            x += delta.x * xSpeed * 0.002f; // Adjust speed factor
            y -= delta.y * ySpeed * 0.002f;
            y = ClampAngle(y, yMinLimit, yMaxLimit);

            lastMousePosition = currentMousePosition; // Update last position
        }

        Quaternion rotation = Quaternion.Euler(y, x, 0);
        cam.transform.rotation = rotation;

        // Pan when left mouse button is pressed
        if (isDragging && Input.GetMouseButton(0))
        {
            Vector3 currentMousePosition = Input.mousePosition;
            Vector3 delta = currentMousePosition - lastMousePosition;

            Vector3 right = cam.transform.right;
            Vector3 up = cam.transform.up;
            target += -delta.x * right * cam.orthographicSize * cameraPanModifier * 0.002f;
            target += -delta.y * up * cam.orthographicSize * cameraPanModifier * 0.002f;

            lastMousePosition = currentMousePosition; // Update last position
        }

        cam.transform.position = target - (rotation * Vector3.forward * 20);

        // Adjust orthographic size with mouse scroll wheel
        if (cam.orthographic && IsMouseOverImage())
        {
            cam.orthographicSize -= Input.GetAxis("Mouse ScrollWheel") * zoomSpeed * cam.orthographicSize;
            cam.orthographicSize = Mathf.Clamp(cam.orthographicSize, sizeMin, sizeMax);
        }
    }

    public static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F)
            angle += 360F;
        if (angle > 360F)
            angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }

    bool IsMouseOverImage()
    {
        if (img == null || canvas == null) return false;
        if (float.IsInfinity(Input.mousePosition.x) || float.IsInfinity(Input.mousePosition.y))
        {
            return false; // Avoid processing when mouse position is invalid
        }

        RectTransform rectTransform = img.rectTransform;
        Vector2 localMousePosition;

        return RectTransformUtility.ScreenPointToLocalPointInRectangle(
            rectTransform,
            Input.mousePosition,
            canvas.renderMode == RenderMode.ScreenSpaceOverlay ? null : Camera.main,
            out localMousePosition)
            && rectTransform.rect.Contains(localMousePosition);
    }
}
