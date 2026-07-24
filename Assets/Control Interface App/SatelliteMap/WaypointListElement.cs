using TMPro;
using Unity.VectorGraphics;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class WaypointListElement : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler
{
    public SVGImage typeIcon;
    public TextMeshProUGUI coordText;

    private Image _background;
    private Color _bgColor;
    [SerializeField] private Color _hoverColor;
    private int _index;
    private WaypointUIController _controller;

    private void Awake()
    {
        _background = GetComponent<Image>();
        _bgColor = _background.color;
        _controller = GetComponentInParent<WaypointUIController>();
    }

    public void Setup(Waypoint wp, int index, Sprite icon)
    {
        _index = index;
        typeIcon.sprite = icon;
        typeIcon.color = Color.white;
        coordText.text = $"{wp.latitude:F5}, <pos=50%>{wp.longitude:F5}";
    }

    public void UpdateCoordinates(double lat, double lon)
    {
        coordText.text = $"{lat:F5}, <pos=50%>{lon:F5}";
    }
    
    public void OnEditClicked()
    {
        if (_controller != null && _controller.editorPanel != null)
        {
            var points = SatelliteMapSystem.Instance.GetWaypoints();
            if (_index < points.Count)
            {
                _controller.editorPanel.OpenForEdit(_index, points[_index].latitude, points[_index].longitude);
            }
        }
    }

    public void OnDeleteClicked()
    {
        SatelliteMapSystem.Instance.RemoveWaypointAtIndex(_index);
    }

    public void OnPointerEnter(PointerEventData eventData = null)
    {
        SatelliteMapSystem.Instance.HighlightWaypoint(_index, true);
        _background.color = _hoverColor;
    }

    public void OnPointerExit(PointerEventData eventData = null)
    {
        SatelliteMapSystem.Instance.HighlightWaypoint(_index, false);
        _background.color = _bgColor;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left)
        {
            SatelliteMapSystem.Instance.CameraFocusWaypoint(_index);
        }
    }
}
