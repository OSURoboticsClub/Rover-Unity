using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using TMPro;

public class sensorDataROS : MonoBehaviour
{
    public GameObject hydrogenTextObject;
    public GameObject ozoneTextObject;
    public GameObject geigerTextObject;

    public GameObject latitudeTextObject;
    public GameObject longitudeTextObject;

    private TextMeshProUGUI hydrogenText;
    private TextMeshProUGUI ozoneText;
    private TextMeshProUGUI geigerText;
    private TextMeshProUGUI latitudeText;
    private TextMeshProUGUI longtitudeText;


    // Shared sensor data, updated by ROS2 callback
    private float hydrogen = 0f;
    private float ozone = 0f;
    private float geiger = 0f;
    private bool newDataAvailable = false;

    private RoverIconController gpsLocation;


    void Start()
    {
        gpsLocation = FindObjectOfType<RoverIconController>();

        hydrogenText = hydrogenTextObject?.GetComponent<TextMeshProUGUI>();
        ozoneText = ozoneTextObject?.GetComponent<TextMeshProUGUI>();
        geigerText = geigerTextObject?.GetComponent<TextMeshProUGUI>();
        latitudeText = latitudeTextObject?.GetComponent<TextMeshProUGUI>();
        longtitudeText = longitudeTextObject?.GetComponent<TextMeshProUGUI>();
    }

    void Update()
    {
        if (newDataAvailable)
        {
            if (hydrogenText != null) hydrogenText.text = $"H2: {hydrogen:F2}";
            if (ozoneText != null) ozoneText.text = $"O3: {ozone:F2}";
            if (geigerText != null) geigerText.text = $"Geiger: {geiger:F2}";


            newDataAvailable = false;
        }
        if (longtitudeText != null) longtitudeText.text = $"Long: {gpsLocation.latestlat}";
        if (latitudeText != null) latitudeText.text = $"Lat: {gpsLocation.latestlon}";
    }
}
