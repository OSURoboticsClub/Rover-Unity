using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverTrail : MonoBehaviour
{
    [SerializeField] LineRenderer line;
    [SerializeField] Transform rover;
    Vector2 lastPointPosition = new Vector2(1000,1000);
    float distance;
    [SerializeField] float cutoff = .1f;

    private void Update()
    {
        float zoomScaler = 1f;
        if (rover.localScale.x < 1) zoomScaler = rover.localScale.x;
        line.startWidth = 0.066f * zoomScaler;
        line.endWidth = 0.066f * zoomScaler;

        distance = Vector2.Distance(lastPointPosition, rover.position);
        if(distance > cutoff)
        {
            lastPointPosition = rover.position;
            line.positionCount++;
            line.SetPosition(line.positionCount - 1, rover.position);
        }
    }

    public void ResetTrail()
    {
        line.positionCount = 0;
    }
}
