using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RescaleGPSCircles : MonoBehaviour
{
    [SerializeField] Transform cirlcesParent;

    // Update is called once per frame
    void Update()
    {
        foreach (Transform child in cirlcesParent)
        {
            float scale = CameraControl.inst.iconScaleForZoom;
            child.localScale = new Vector3(scale, scale, 1);
        }
    }
}
