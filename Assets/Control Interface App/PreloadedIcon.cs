using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PreloadedIcon : MonoBehaviour
{
    public double latitude;
    public double longitude;
    public string description;

    public void Start()
    {
        MapController.instance.MoveIcon(transform, latitude, longitude);
        LocationsListController.inst.CreateNewListing(this);
    }
}
