using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using Unity.VisualScripting.FullSerializer;
using UnityEditor.PackageManager;
using UnityEditor.VersionControl;
using UnityEngine;

public enum TowerErrorCodes
{
    NO_COMM_ELEV_AXIS      = 0b0000_0000_0000_0001,
    NO_COMM_PAN_AXIS       = 0b0000_0000_0000_0010,
    TIMEOUT_ROVER_GPS      = 0b0000_0000_0000_0100,
    TIMEOUT_ROVER_IMU      = 0b0000_0000_0000_1000,
    TIMEOUT_TOWER_GPS      = 0b0000_0000_0001_0000,
    TIMEOUT_TOWER_IMU      = 0b0000_0000_0010_0000,
    AXIS_UNCALIBRATED_PAN  = 0b0000_0000_0100_0000,
    AXIS_UNCALIBRATED_ELEV = 0b0000_0000_1000_0000
}


public class TowerErrorCodeDisplay : MonoBehaviour
{
    [SerializeField] private TowerSubscriber towerSubscriber;
    public List<string> ErrorMessages = new List<string> {
        "No communications on Tilt Axis",
        "No communications on Pan Axis",
        "Rover GPS Timout",
        "Rover IMU Timout",
        "Tower GPS Timout",
        "Tower IMU Timout",
        "Pan Axis uncalibrated",
        "Tilt Axis uncalibrated",
        };
    public int lastError = -1;
    [SerializeField] TMP_Text errorOutput;

    // Start is called before the first frame update
    void Start()
    {
        towerSubscriber.onAntennaControlStatusUpdate.AddListener(antennaControlStatusUpdate);
        
        antennaControlStatusUpdate(towerSubscriber.currentStatus);
    }
    string getErrorMessage(TowerErrorCodes code)
    {
        return code switch {
            TowerErrorCodes.NO_COMM_ELEV_AXIS => "No communications on Tilt Axis",
            TowerErrorCodes.NO_COMM_PAN_AXIS => "No communications on Pan Axis",
            TowerErrorCodes.TIMEOUT_ROVER_GPS => "Rover GPS Timout",
            TowerErrorCodes.TIMEOUT_ROVER_IMU => "Rover IMU Timout",
            TowerErrorCodes.TIMEOUT_TOWER_GPS => "Tower GPS Timout",
            TowerErrorCodes.TIMEOUT_TOWER_IMU => "Tower IMU Timout",
            TowerErrorCodes.AXIS_UNCALIBRATED_PAN => "Pan Axis uncalibrated",
            TowerErrorCodes.AXIS_UNCALIBRATED_ELEV => "Tower IMU Timout",
            _ => "Unknown Error"
        };
    }

    public void antennaControlStatusUpdate(AntennaControlStatus newStatus)
    {
        if(newStatus.errors == lastError) return;
        string newText = "";
        foreach (TowerErrorCodes errorCode in Enum.GetValues(typeof(TowerErrorCodes)))
        {
            int result = (int)errorCode & (int)newStatus.errors;
            if(result == (int)errorCode)
            {
                newText += getErrorMessage(errorCode) + "\n";
            }
        }
        errorOutput.text = newText;
    }

}
