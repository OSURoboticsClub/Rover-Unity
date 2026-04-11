using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class StatusIndicator : MonoBehaviour
{
    [SerializeField] Color success;
    [SerializeField] Color warning;
    [SerializeField] Color failure;
    [SerializeField] TextMeshProUGUI text;

    private Image bg;
    private bool _isSubscribed = false;

    void OnEnable()
    {
        TrySubscribe();
    }

    void Start()
    {
        TrySubscribe();
        bg = GetComponentInChildren<Image>();
    }

    private void TrySubscribe()
    {
        if (_isSubscribed || SatelliteMapSystem.Instance == null)
            return;
        SatelliteMapSystem.Instance.OnTelemetryReceived += OnTelemetry;
        _isSubscribed = true;
    }

    void OnDisable()
    {
        if (SatelliteMapSystem.Instance != null)
            SatelliteMapSystem.Instance.OnTelemetryReceived -= OnTelemetry;
            _isSubscribed = false;
    }

    public void OnTelemetry(MissionTelemetry telemetry)
    {
        MissionState state = telemetry.mission_state;
        string stateStr = state.ToString().Replace("_", " ").ToLower();
        text.text = stateStr;
        bg.color = state switch
        {
            MissionState.IDLE => failure,
            MissionState.MOVING_TO_START => warning,
            MissionState.SEARCHING => warning,
            MissionState.WAITING_FOR_NAV_IDLE => warning,
            MissionState.INVESTIGATING => warning,
            MissionState.RETURNING_TO_SEARCH => warning,
            MissionState.SUCCESS => success,
            MissionState.FAILED => failure,
            MissionState.RETURNING_HOME => warning,
            MissionState.STOPPED => failure,
            _ => failure
        };
    }
}
