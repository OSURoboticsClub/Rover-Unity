using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class GpsLocation : MonoBehaviour
{
    public string description;
    public double latitude;
    public double longitude;
    public bool isActive;

    public TextMeshProUGUI desc;
    public TextMeshProUGUI lat;
    public TextMeshProUGUI lon;
    public TextMeshProUGUI waypointCountText;
    public int waypointCount;

    [SerializeField] Image goBtn;
    [SerializeField] TextMeshProUGUI goTxt;
    public GameObject iconObject;

    public void UpdateLabels()
    {
        desc.text = description;
        lat.text = latitude.ToString();
        lon.text = longitude.ToString();
        waypointCountText.text = waypointCount.ToString();
    }

    public void SetActive()
    {
        goBtn.color = new Color(255 / 255f, 143 / 255f, 151 / 255f);
        goTxt.text = "Stop";
    }

    public void SetInactive()
    {
        goBtn.color = new Color(143 / 255f, 1f, 143 / 255f);
        goTxt.text = "Go";
        MapController.instance.TurnOffLine();
    }

    public void ToggleActive()
    {
        CurrentDestinationController.inst.ClickBtn(this);
    }

    public void Delete()
    {
        Destroy(iconObject);
        Destroy(gameObject);
    }
}
