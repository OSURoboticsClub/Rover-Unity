using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmCameraOrbit : MonoBehaviour
{
    public Transform target;  // The object to orbit around
    public Transform cam;  // The object to orbit around
    public float rotationSpeed = 5f;
    public float minPitch = -80f;
    public float maxPitch = 80f;
    public float distance = 5f;

    float yaw = 0f;
    float pitch = 0f;

    void Start()
    {
        if (target == null)
        {
            Debug.LogError("No target assigned to orbit around!");
            return;
        }

        // Initialize yaw and pitch based on current camera position
        Vector3 angles = cam.eulerAngles;
        yaw = angles.y;
        pitch = angles.x;
    }

    void Update()
    {
        if (target == null) return;

        if (Input.GetMouseButton(1)) // Right Mouse Button
        {
            float horizontal = Input.GetAxis("Mouse X") * rotationSpeed;
            float vertical = -Input.GetAxis("Mouse Y") * rotationSpeed; // invert Y

            yaw += horizontal;
            pitch += vertical;
            pitch = Mathf.Clamp(pitch, minPitch, maxPitch); // limit vertical angle
        }

        // Convert yaw/pitch into a rotation
        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);

        // Compute new camera position based on that rotation
        Vector3 newPos = target.position + (rotation * Vector3.back * distance);

        cam.position = newPos;
        cam.LookAt(target);
    }
}
