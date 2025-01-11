using UnityEngine;

public class DifferentialDrive : MonoBehaviour
{
    public float wheelBase = 0.5f; // Distance between wheels
    public float wheelRadius = 0.1f; // Radius of the wheels
    public float maxVoltage = 12f; // Maximum voltage
    public float maxAngularVelocity = 10f; // Max angular velocity (rad/s)

    public static float leftVoltage;
    public static float rightVoltage;
    public float leftVDisplay;
    public float rightVDisplay;

    private float x = 0f, y = 0f, theta = 0f; // Position and orientation

    private void Start()
    {
        x = transform.position.x;
        y = transform.position.y;
    }

    void Update()
    {
        //leftVoltage = leftVDisplay;
        //rightVoltage = rightVDisplay;
        float deltaTime = Time.deltaTime;

        // Convert voltages to angular velocities
        float leftAngularVelocity = (leftVoltage / maxVoltage) * maxAngularVelocity;
        float rightAngularVelocity = (rightVoltage / maxVoltage) * maxAngularVelocity;

        // Convert angular velocities to linear velocities
        float leftLinearVelocity = leftAngularVelocity * wheelRadius;
        float rightLinearVelocity = rightAngularVelocity * wheelRadius;

        // Compute linear and angular velocities of the robot
        float v = (leftLinearVelocity + rightLinearVelocity) / 2f;
        float omega = (rightLinearVelocity - leftLinearVelocity) / wheelBase;

        // Update position and orientation
        float deltaTheta = omega * deltaTime;
        theta += deltaTheta;

        float deltaX = v * Mathf.Sin(-theta) * deltaTime;
        float deltaY = v * Mathf.Cos(theta) * deltaTime;

        x += deltaX;
        y += deltaY;

        // Apply to the GameObject
        transform.position = new Vector3(x, y, transform.position.z);
        transform.rotation = Quaternion.Euler(0, 0, Mathf.Rad2Deg * theta);
        leftVDisplay = leftVoltage;
        rightVDisplay = rightVoltage;
    }
}
