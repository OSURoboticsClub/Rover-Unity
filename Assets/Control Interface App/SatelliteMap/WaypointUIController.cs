using UnityEngine;
using UnityEngine.UI;

public class WaypointUIController : MonoBehaviour
{
    public GameObject waypointElementPrefab;
    public Transform contentContainer;
    public GameObject instructionText;

    [Header("Marker Icons")]
    public Sprite roverMarker;
    public Sprite startMarker;
    public Sprite waypointMarker;
    public Sprite goalMarker;
    public Sprite searchBoundsMarker;

    public Sprite bottleMarker;
    public Sprite hammerMarker;
    public Sprite malletMarker;
    public Sprite arucoMarker;

    public WaypointEditorPanel editorPanel;
    public Button addWaypointButton;

    private bool _isSubscribed = false;

    private void Awake()
    {
        if (editorPanel != null)
            editorPanel.gameObject.SetActive(false);
        if (addWaypointButton != null)
            addWaypointButton.onClick.AddListener(OnAddWaypointClicked);
    }

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
            SatelliteMapSystem.Instance.OnWaypointHoverStart -= HighlightListRow;
            SatelliteMapSystem.Instance.OnWaypointHoverEnd -= UnhighlightListRow;
            SatelliteMapSystem.Instance.OnWaypointDragged -= UpdateWaypointDragged;
            _isSubscribed = false;
        }
    }

    private void TrySubscribe()
    {
        if (_isSubscribed || SatelliteMapSystem.Instance == null)
            return;
        SatelliteMapSystem.Instance.OnWaypointsChanged += RefreshList;
        SatelliteMapSystem.Instance.OnWaypointHoverStart += HighlightListRow;
        SatelliteMapSystem.Instance.OnWaypointHoverEnd += UnhighlightListRow;
        SatelliteMapSystem.Instance.OnWaypointDragged += UpdateWaypointDragged;
        _isSubscribed = true;
    }

    public void OnAddWaypointClicked()
    {
        if (editorPanel != null) 
            editorPanel.OpenForAdd();
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
            instructionText.SetActive(points.Count == 0);

        for (int i = 0; i < points.Count; i++)
        {
            var marker = GetMarker(i, points.Count, searchObject);
            GameObject go = Instantiate(waypointElementPrefab, contentContainer);
            go.GetComponent<WaypointListElement>().Setup(points[i], i, marker);
        }
    }

    private void HighlightListRow(int index)
    {
        if (index < contentContainer.childCount)
        {
            var element = contentContainer.GetChild(index).GetComponent<WaypointListElement>();
            if (element != null)
                element.OnPointerEnter(null);
        }
    }
    
    private void UnhighlightListRow(int index)
    {
        if (index >= 0 && index < contentContainer.childCount)
        {
            var element = contentContainer.GetChild(index).GetComponent<WaypointListElement>();
            if (element != null)
                element.OnPointerExit(null);
        }
    }

    private Sprite GetMarker(int index, int count, MissionConfig.SearchObject searchObject)
    {
        if (index == 0) return roverMarker;
        if (index == count - 1)
        {
            return searchObject switch
            {
                MissionConfig.SearchObject.Waterbottle => bottleMarker,
                MissionConfig.SearchObject.Hammer => hammerMarker,
                MissionConfig.SearchObject.Mallet => malletMarker,
                MissionConfig.SearchObject.Aruco => arucoMarker,
                _ => goalMarker
            };
        }
        return waypointMarker;
    }

    private void UpdateWaypointDragged(int index, Waypoint newWaypoint)
    {
        if (index >= 0 && index < contentContainer.childCount)
        {
            var element = contentContainer.GetChild(index).GetComponent<WaypointListElement>();
            if (element != null)
                element.UpdateCoordinates(newWaypoint.latitude, newWaypoint.longitude);
        }
    }
}
