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
    public Image rowIcon;

    [SerializeField] Image goBtn;
    [SerializeField] TextMeshProUGUI goTxt;
    public LineRenderer line;
    public GameObject iconObject;
    public List<GameObject> waypoints = new();
    public ItemToFind itemAtDestination;


    private void Update()
    {
        if (line.enabled)
        {
            line.widthMultiplier = 2f * CameraControl.inst.secondCamera.orthographicSize * CameraControl.inst.lineScale;
        }
    }

    public void UpdateLabels()
    {
        desc.text = description;
        lat.text = latitude.ToString();
        lon.text = longitude.ToString();
        waypointCountText.text = waypoints.Count.ToString();
    }

    public void ClearWaypoints()
    {
        foreach(var x in waypoints)
        {
            Destroy(x);
        }
        waypoints.Clear();
        waypointCountText.text = "0";
    }

    public void AddWaypoint(GameObject newObject)
    {
        waypoints.Add(newObject);
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

    public void ReturnToOrigin()
    {
        CurrentDestinationController.inst.ReturnToStart();
    }

    public void SetInactiveUI()
    {
        goBtn.color = new Color(143 / 255f, 1f, 143 / 255f);
        goTxt.text = "Go";
    }

    public void ToggleActive()
    {
        CurrentDestinationController.inst.ClickGoButton(this);
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
