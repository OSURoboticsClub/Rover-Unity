using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class SetWaypointButtonClick : MonoBehaviour
{
    public bool isModifying = false;
    [SerializeField] TextMeshProUGUI btnText;
    [SerializeField] Image btnColor;

    public void OnClick()
    {
        isModifying = !isModifying;

        if (isModifying)
        {
            btnText.text = "Finish";
            btnColor.color = new Color(255 / 255f, 131 / 255f, 136 / 255f);
            GpsLocation locationScript = transform.GetComponentInParent<GpsLocation>();
            WaypointAdder.inst.OpenAddingMode(locationScript);
        }
        else
        {
            btnText.text = "Set Waypts";
            btnColor.color = new Color(115 / 255f, 232 / 255f, 255 / 255f);
            WaypointAdder.inst.CloseAddingMode();
        }
    }
}
