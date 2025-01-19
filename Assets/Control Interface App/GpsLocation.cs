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

    [SerializeField] Image goBtn;
    [SerializeField] TextMeshProUGUI goTxt;
    public GameObject iconObject;

    public void UpdateLabels()
    {
        desc.text = description;
        lat.text = latitude.ToString();
        lon.text = longitude.ToString();
    }

    public void SetActive(bool on)
    {
        isActive = on;

        if (isActive)
        {
            //change button to red
            goBtn.color = new Color(255 / 255f, 143/255f, 151 / 255f);
            goTxt.text = "Stop";

            string message = $"auton_control;GOTO;{lat.text};{lon.text}";
            TcpPublish.inst.Publish(message);
            StatusIndicator.instance.SetIndicator(Status.Activated, this);
            MapController.instance.SetLinePosition(iconObject.transform.position);
        }
        else
        {
            //change button to green
            goBtn.color = new Color(143 / 255f, 1f, 143 / 255f);
            goTxt.text = "Go";
            string message = $"auton_control;STOP;{lat.text};{lon.text}";
            TcpPublish.inst.Publish(message);
            StatusIndicator.instance.SetIndicator(Status.NotActivated, this);
            MapController.instance.TurnOffLine();
        }
    }

    public void ToggleActive()
    {
        SetActive(!isActive);
    }

    public void Delete()
    {
        Destroy(iconObject);
        Destroy(gameObject);
    }
}
