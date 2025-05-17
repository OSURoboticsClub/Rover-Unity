using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocationsListController : MonoBehaviour
{
    // sorry this has gotten so confusing
    public static LocationsListController inst;
    [SerializeField] GameObject listingObject;
    [SerializeField] Transform listingParent;
    [SerializeField] Transform newListingButton;
    [SerializeField] Sprite aruco;
    [SerializeField] Sprite hammer;

    private void Awake()
    {
        inst = this;
    }

    public void CreateNewListing(PreloadedIcon icon)
    {
        GameObject newObject = Instantiate(listingObject, listingParent);
        var locationScript = newObject.GetComponent<GpsLocation>();
        locationScript.description = icon.description;
        locationScript.latitude = icon.latitude;
        locationScript.longitude = icon.longitude;
        locationScript.iconObject = icon.gameObject;
        locationScript.rowIcon.sprite = icon.icon;
        locationScript.itemAtDestination = ItemToFind.none;
        if(icon.icon == aruco) locationScript.itemAtDestination = ItemToFind.aruco;
        else if(icon.icon == hammer) locationScript.itemAtDestination = ItemToFind.hammer;
        locationScript.UpdateLabels();
        newListingButton.SetAsLastSibling();
    }
}
