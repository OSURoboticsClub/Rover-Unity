using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public enum Status {
    Activated,
    NotActivated,
    Arrived,
    Error
}

public class StatusIndicator : MonoBehaviour
{
    public static StatusIndicator instance;
    [SerializeField] Image bg;
    [SerializeField] Color red;
    [SerializeField] Color green;
    [SerializeField] TextMeshProUGUI text;
    GpsLocation destination;

    // Start is called before the first frame update
    void Start()
    {
        instance = this;
    }

    public void SetIndicator(Status status, GpsLocation location){
        if(status == Status.Activated){
            bg.color = red;
            text.text = "GOING TO " + location.desc.text;
            destination = location;
        }
        else if(status == Status.NotActivated){
            bg.color = green;
            text.text = "NOT ACTIVATED";
        }
    }
}
