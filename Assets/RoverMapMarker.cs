using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverMapMarker : MonoBehaviour
{

    public float smoothSpeed = 10f;
    private Vector3 _targetPosition;
    private Quaternion _targetRotation;

    void OnEnable()
    {
        if (SatelliteMapSystem.Instance != null)
            SatelliteMapSystem.Instance.OnTelemetryReceived += OnTelemetry;
    }
    void OnDisable()
    {
        if (SatelliteMapSystem.Instance != null)
            SatelliteMapSystem.Instance.OnTelemetryReceived -= OnTelemetry;
    }

    void Start()
    {
        _targetPosition = transform.position;
        _targetRotation = transform.rotation;

        SpriteRenderer sr = GetComponent<SpriteRenderer>();
        sr.color = new Color(243f / 255f, 208f / 255f, 63f / 255f);
        sr.material = new Material(sr.sharedMaterial);
        sr.material.SetColor("_Color", sr.color);
    }

    void Update()
    {
        transform.position = Vector3.Lerp(
            transform.position,
            _targetPosition,
            Time.deltaTime * smoothSpeed
        );

        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            _targetRotation,
            Time.deltaTime * smoothSpeed
        );
    }

    public void OnTelemetry(MissionTelemetry telemetry)
    {
        Vector3 roverPos = SatelliteMapSystem.Instance.GetUnityPositionFromGPS(telemetry.latitude, telemetry.longitude);
        roverPos.y = 0.1f;
        Quaternion roverRot = Quaternion.Euler(90f, 0f, telemetry.heading - 90f);
        _targetPosition = roverPos;
        _targetRotation = roverRot;
    }
}
