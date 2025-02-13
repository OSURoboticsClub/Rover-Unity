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

    [SerializeField] Image goBtn;
    [SerializeField] TextMeshProUGUI goTxt;
    public LineRenderer line;
    public GameObject iconObject;
    public List<GameObject> waypoints = new();

    public void UpdateLabels()
    {
        desc.text = description;
        lat.text = latitude.ToString();
        lon.text = longitude.ToString();
        waypointCountText.text = waypoints.Count.ToString();
    }

    public void SetActive()
    {
        goBtn.color = new Color(255 / 255f, 143 / 255f, 151 / 255f);
        goTxt.text = "Stop";
    }

    public void SetHighlightStatus(bool status)
    {
        foreach(var x in waypoints)
        {
            x.SetActive(status);
        }
        line.enabled = status;
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

    public void SetLine()
    {
        line.positionCount = waypoints.Count + 1;
        for (int i = 0; i < waypoints.Count; i++)
        {
            line.SetPosition(i, waypoints[i].transform.position);
        }
        line.SetPosition(waypoints.Count, iconObject.transform.position);
    }
}
