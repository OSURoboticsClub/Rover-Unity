using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class WaypointListElement : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler
{
    public Image typeIcon;
    public TextMeshProUGUI coordText;

    private Image _background;
    private Color _bgColor;
    [SerializeField] private Color _hoverColor;
    private int _index;

    private void Awake()
    {
        _background = GetComponent<Image>();
        _bgColor = _background.color;
    }

    public void Setup(Waypoint wp, int index, Sprite icon)
    {
        _index = index;
        typeIcon.sprite = icon;
        typeIcon.color = Color.white;
        coordText.text = $"{wp.latitude:F5}, <pos=50%>{wp.longitude:F5}";
    }

    public void OnDeleteClicked()
    {
        SatelliteMapSystem.Instance.RemoveWaypointAtIndex(_index);
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        SatelliteMapSystem.Instance.HighlightWaypoint(_index, true);
        _background.color = _hoverColor;
    }

    // --- HOVER END ---
    public void OnPointerExit(PointerEventData eventData)
    {
        SatelliteMapSystem.Instance.HighlightWaypoint(_index, false);
        _background.color = _bgColor;
    }

    // --- CLICK ---
    public void OnPointerDown(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left)
        {
            SatelliteMapSystem.Instance.CameraFocusWaypoint(_index);
        }
    }
}
