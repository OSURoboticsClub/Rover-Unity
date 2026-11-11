using UnityEngine;
using TMPro;

public class Towermodeswitcher : MonoBehaviour
{
    [SerializeField] private TMP_Text manualButtonText;
    [SerializeField] private TMP_Text automaticButtonText;
    [SerializeField] private TMP_Text homingButtonText;
    [SerializeField] private TMP_Text disabledButtonText;

    public void activate_buttons(string modeName)
    {
        TMP_Text buttonText = modeName switch
        {
            "Manual" => manualButtonText,
            "Automatic" => automaticButtonText,
            "Homing" => homingButtonText,
            "Disabled" => disabledButtonText,
            _ => null
        };

        if (buttonText == null)
        {
            Debug.LogError($"No button text is configured for mode '{modeName}'.", this);
            return;
        }

        buttonText.text = modeName + " mode activated";
    }
}
