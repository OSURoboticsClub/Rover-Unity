using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmCameraOrbit : MonoBehaviour
{
    public Vector3 target; // The target vector to orbit and pan around

    [SerializeField] Camera cam; // Camera component
    [SerializeField] Transform arm;
    public float xSpeed = 120.0f;
    public float ySpeed = 120.0f;

    public float cameraPanModifier = 0.1f;

    public float yMinLimit = -20f; // Minimum vertical angle
    public float yMaxLimit = 80f; // Maximum vertical angle

    public float sizeMin = 1f; // Minimum orthographic size
    public float sizeMax = 20f; // Maximum orthographic size

    float x = 0.0f;
    float y = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        Vector3 angles = cam.transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        target = arm.position;
    }

    void LateUpdate()
    {
        if (Input.GetMouseButton(1)) // Only rotate when the right mouse button is held down
        {
            x += Input.GetAxis("Mouse X") * xSpeed * 0.02f;
            y -= Input.GetAxis("Mouse Y") * ySpeed * 0.02f;

            y = ClampAngle(y, yMinLimit, yMaxLimit);
        }

        Quaternion rotation = Quaternion.Euler(y, x, 0);
        cam.transform.rotation = rotation;

        if (Input.GetMouseButton(2)) // Pan when middle mouse button is pressed
        {
            Vector3 right = cam.transform.right;
            Vector3 up = cam.transform.up;
            target += -Input.GetAxis("Mouse X") * right * cam.orthographicSize * cameraPanModifier;
            target += -Input.GetAxis("Mouse Y") * up * cam.orthographicSize * cameraPanModifier;
        }

        cam.transform.position = target - (rotation * Vector3.forward * 20);

        // Adjust orthographic size with mouse scroll wheel
        if (cam.orthographic)
        {
            cam.orthographicSize -= Input.GetAxis("Mouse ScrollWheel") * 5f;
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
}
