using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SelectDestination : MonoBehaviour
{
    [SerializeField] Image bg;
    public void SetBgColor()
    {
        bg.color = new Color(0, 134/255f, 1f, 33 / 255f);
        SetIconGreen();

        Transform listParent = transform.parent;
        foreach(Transform child in listParent)
        {
            if (child.gameObject.name == "Button") continue;
            var scr = child.GetComponent<SelectDestination>();
            if (scr == this) continue;
            scr.ResetColor();
            scr.SetIconBlue();
        }
    }

    public void ResetColor()
    {
        bg.color = new Color(0, 1f, 14 / 255f, 0);
    }

    public void SetIconBlue()
    {
        var scr = GetComponent<GpsLocation>();
        scr.iconObject.transform.GetChild(0).GetComponent<SpriteRenderer>().color = new Color(1, 1, 1);
    }

    public void SetIconGreen()
    {
        var scr = GetComponent<GpsLocation>();
        scr.iconObject.transform.GetChild(0).GetComponent<SpriteRenderer>().color = new Color(0, 1f, 2/255f);
    }
}
