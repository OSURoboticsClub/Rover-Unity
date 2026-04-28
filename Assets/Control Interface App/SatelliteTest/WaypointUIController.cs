using UnityEngine;

public class WaypointUIController : MonoBehaviour
{
    public GameObject waypointElementPrefab;
    public Transform contentContainer;
    public GameObject instructionText;

    [Header("Marker Icons")]
    public Sprite roverMarker;
    public Sprite waypointMarker;
    public Sprite goalMarker;
    public Sprite bottleMarker;
    public Sprite hammerMarker;
    public Sprite malletMarker;
    public Sprite arucoMarker;

    private bool _isSubscribed = false;

    private void Start()
    {
        TrySubscribe();
        RefreshList();
    }

    private void OnEnable()
    {
        TrySubscribe();
        RefreshList();
    }

    private void OnDisable()
    {
        if (SatelliteMapSystem.Instance != null)
        {
            SatelliteMapSystem.Instance.OnWaypointsChanged -= RefreshList;
            _isSubscribed = false;
        }
    }

    private void TrySubscribe()
    {
        if (_isSubscribed || SatelliteMapSystem.Instance == null)
            return;
        SatelliteMapSystem.Instance.OnWaypointsChanged += RefreshList;
        _isSubscribed = true;
    }

    public void RefreshList()
    {
        if (SatelliteMapSystem.Instance == null)
            return;

        foreach (Transform child in contentContainer)
            Destroy(child.gameObject);

        var searchObject = SatelliteMapSystem.Instance.GetSearchObject();
        var points = SatelliteMapSystem.Instance.GetWaypoints();

        if (instructionText != null)
        {
            instructionText.SetActive(points.Count == 0);
        }

        for (int i = 0; i < points.Count; i++)
        {
            var marker = GetMarker(i, points.Count, searchObject);
            GameObject go = Instantiate(waypointElementPrefab, contentContainer);
            go.GetComponent<WaypointListElement>().Setup(points[i], i, marker);
        }
    }

    private Sprite GetMarker(int index, int count, MissionConfig.SearchObject searchObject)
    {
        if (index == 0) return roverMarker;
        if (index == count - 1)
        {
            return searchObject switch
            {
                MissionConfig.SearchObject.Bottle => bottleMarker,
                MissionConfig.SearchObject.Hammer => hammerMarker,
                MissionConfig.SearchObject.Mallet => malletMarker,
                MissionConfig.SearchObject.Aruco => arucoMarker,
                _ => goalMarker
            };
        }
        return waypointMarker;
    }
}
