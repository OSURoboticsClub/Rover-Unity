using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClosenessToCoordinateBar : MonoBehaviour
{
    [SerializeField] float closenessDist;
    [SerializeField] float maxDist;
    // Start is called before the first frame update
    void Start()
    {
        transform.localScale = new Vector3(1f, 0f, 1f);
    }

    // Update is called once per frame
    void Update()
    {
        float yScale = 1f;
        if (CurrentDestinationController.inst.distToTarget > maxDist) yScale = 0f;
        else if (CurrentDestinationController.inst.distToTarget < closenessDist) yScale = 1f;
        else
        {
            float range = maxDist - closenessDist;
            yScale = 1 - (CurrentDestinationController.inst.distToTarget - closenessDist) / range;
        }
        transform.localScale = new Vector3(1f, yScale, 1f);
    }
}
