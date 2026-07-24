using UnityEngine;
using TMPro;
using UnityEngine.UI;
using Unity.VisualScripting;

public class WaypointEditorPanel : MonoBehaviour
{
    public TMP_Text headerText;
    public TMP_InputField latInput;
    public TMP_InputField lonInput;
    public Button saveButton;
    public Button cancelButton;

    private bool _isEditing;
    private int _editIndex;


    private void Awake()
    {
        if (saveButton != null)
            saveButton.onClick.AddListener(OnSaveClicked);
        if (cancelButton != null)
            cancelButton.onClick.AddListener(OnCancelClicked);
    }

    public void OpenForAdd()
    {
        _isEditing = false;
        latInput.text = "";
        lonInput.text = "";
        headerText.text = "Add Waypoint";
        gameObject.SetActive(true);
    }

    public void OpenForEdit(int index, double lat, double lon)
    {
        _isEditing = true;
        _editIndex = index;
        latInput.text = lat.ToString("F7");
        lonInput.text = lon.ToString("F7");
        headerText.text = "Edit Waypoint";
        gameObject.SetActive(true);
    }

    public void OnSaveClicked()
    {
        if (double.TryParse(latInput.text, out double lat) && double.TryParse(lonInput.text, out double lon))
        {
            if (_isEditing)
                SatelliteMapSystem.Instance.UpdateWaypointAtIndex(_editIndex, lat, lon);
            else
                SatelliteMapSystem.Instance.AddWaypoint(lat, lon);

            gameObject.SetActive(false);
            return;
        }

        Debug.LogWarning("Invalid GPS coordinates entered.");
    }

    public void OnCancelClicked()
    {
        gameObject.SetActive(false);
    }

}