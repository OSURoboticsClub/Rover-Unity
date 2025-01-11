using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class GpsLocation : MonoBehaviour
{
    public string description;
    public decimal latitude;
    public decimal longitude;
    public bool isActive;

    [SerializeField] TextMeshProUGUI desc;
    [SerializeField] TextMeshProUGUI lat;
    [SerializeField] TextMeshProUGUI lon;

    [SerializeField] Image goBtn;
    [SerializeField] TextMeshProUGUI goTxt;

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
        }
        else
        {
            //change button to green
            goBtn.color = new Color(143 / 255f, 1f, 143 / 255f);
            goTxt.text = "Go";
        }
    }

    public void ToggleActive()
    {
        SetActive(!isActive);
    }

    public void Delete()
    {
        Destroy(gameObject);
    }
}
