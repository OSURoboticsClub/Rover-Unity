using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PurePursuit : MonoBehaviour
{
    [SerializeField] LineRenderer line;
    [SerializeField] LineRenderer pathTaken;
    [SerializeField] Transform pointParent;
    public List<Vector3> Waypoints = new();
    public float LookaheadDistance;
    [SerializeField] int currentWaypointIndex = 0;
    [SerializeField] float currentDistance;
    [SerializeField] float robotTheta;
    [SerializeField] float pathCurvature;
    [SerializeField] float angularVel;
    [SerializeField] float angleToPoint;
    [SerializeField] float localXd;
    [SerializeField] float localYd;

    private void Start()
    {
        foreach(Transform child in pointParent)
        {
            Waypoints.Add(new Vector3(child.transform.position.x, child.transform.position.y, 0));
        }
    }

    private void Update()
    {
        var pos = transform.position;
        pos.z = 0;
        line.SetPosition(0, pos);
        line.SetPosition(1, Waypoints[currentWaypointIndex]);

        currentDistance = Vector2.Distance(pos, Waypoints[currentWaypointIndex]);
        robotTheta = transform.eulerAngles.z * Mathf.Deg2Rad;
        if (robotTheta > Mathf.PI) robotTheta = robotTheta - Mathf.PI * 2;
        robotTheta *= -1;

        float curvature = CalculateCurvature(new Vector2(pos.x, pos.y), robotTheta, Waypoints[currentWaypointIndex]);
        float linearVelocity = 6.0f; // Constant forward velocity
        float angularVelocity = curvature * linearVelocity;

        pathCurvature = curvature;
        angularVel = angularVelocity;


        // Calculate wheel velocities
        float leftV = linearVelocity + angularVelocity * 0.5f; // Adjust for robot width
        float rightV = linearVelocity - angularVelocity * 0.5f;
        DifferentialDrive.leftVoltage = leftV;
        DifferentialDrive.rightVoltage = rightV;


        if (currentDistance < LookaheadDistance) currentWaypointIndex++;
        AddPointToLine(pathTaken, new Vector3(transform.position.x, transform.position.y, 0));
    }

    private float CalculateCurvature(Vector2 robotPosition, float robotTheta, Vector2 lookaheadPoint)
    {
        // Calculate the direction vector to the lookahead point
        Vector2 direction = lookaheadPoint - robotPosition;
        angleToPoint = Vector2.Angle(Vector2.up, direction);

        float a = Mathf.Atan(direction.y / direction.x);
        float hypotenuse = Mathf.Sqrt(direction.y * direction.y + direction.x * direction.x);

        // Rotate direction vector into robot's local frame
        float localX = hypotenuse * Mathf.Cos(a + robotTheta);
        float localY = Mathf.Sqrt(hypotenuse * hypotenuse - localX * localX);
        if (direction.x < 0) localX *= -1;

        if (localY == 0) return 0;
        return 2 * localX / (localX * localX + localY * localY); // Curvature is 2 * x / (x^2 + y^2)
    }

    public void AddPointToLine(LineRenderer line, Vector3 newPoint)
    {
        // Get the current number of points in the LineRenderer
        int pointCount = line.positionCount;

        // Increase the point count by 1
        line.positionCount = pointCount + 1;

        // Set the new point at the end of the LineRenderer
        line.SetPosition(pointCount, newPoint);
    }

}
