using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocationsListController : MonoBehaviour
{
    public static LocationsListController inst;
    [SerializeField] GameObject listingObject;
    [SerializeField] Transform listingParent;
    [SerializeField] Transform newListingButton;

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
        locationScript.UpdateLabels();
        newListingButton.SetAsLastSibling();
    }
}
