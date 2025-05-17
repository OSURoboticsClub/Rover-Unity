using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PreloadedIcon : MonoBehaviour
{
    public double latitude;
    public double longitude;
    public string description;
    public Sprite icon;

    public void Start()
    {
        icon = transform.GetChild(0).GetComponent<SpriteRenderer>().sprite;
        MapController.instance.MoveIcon(transform, latitude, longitude);
        LocationsListController.inst.CreateNewListing(this);
    }
}
