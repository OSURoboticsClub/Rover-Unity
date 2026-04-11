using UnityEngine;
using UnityEngine.UI;

public class ObjectButtonHandler : MonoBehaviour
{
    public ObjectType objectType;
    [SerializeField] private Image backgroundImage;

    private bool isSelected = false;
    private Color ON_COLOR = new Color (25f / 255f, 87f / 255f, 25f / 255f);
    private Color OFF_COLOR = new Color(38f / 255f, 38f / 255f, 38f / 255f);

    private void UpdateVisual()
    {
        backgroundImage.color = isSelected ? ON_COLOR : OFF_COLOR;
    }

    public void SetSelected(bool selected)
    {
        isSelected = selected;
        UpdateVisual();
    }

    public void OnClick()
    {
        isSelected = !isSelected;
        UpdateVisual();
        MapController.instance.OnObjectButtonClicked(objectType, isSelected);
    }
}