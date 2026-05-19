using UnityEngine;
using UnityEngine.UI;

public class ObjectButtonHandler : MonoBehaviour
{
    public ObjectType objectType;
    [SerializeField] private Image backgroundImage;

    public MissionConfig.SearchObject searchObject;
    private Color ON_COLOR = new Color (25f / 255f, 87f / 255f, 25f / 255f);
    private Color OFF_COLOR = new Color(38f / 255f, 38f / 255f, 38f / 255f);

    private Button _button;

    public void Awake() {
        _button = GetComponent<Button>();
       // _button.onClick.AddListener(OnClick);
        backgroundImage.color = OFF_COLOR;
    }


    public void UpdateVisual(bool isSelected)
    {
        Debug.Log($"Updating visual for {objectType} to {(isSelected ? "ON" : "OFF")}");
        backgroundImage.color = isSelected ? ON_COLOR : OFF_COLOR;
    }

    public void OnClick()
    {
        MapController.instance.OnObjectButtonClicked(this);
    }
}
