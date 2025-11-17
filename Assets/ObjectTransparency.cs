using UnityEngine;
using UnityEngine.UI;

public class SpriteTransparency : MonoBehaviour
{
    public Slider slider;
    private SpriteRenderer sr;

    void Start()
    {
        sr = GetComponent<SpriteRenderer>();  // get sprite renderer

        slider.onValueChanged.AddListener(SetAlpha);

        // Initialize full opacity
        var c = sr.color;
        c.a = slider.value;
        sr.color = c;
    }

    void SetAlpha(float a)
    {
        var c = sr.color;
        c.a = a;    // 0–1
        sr.color = c;
    }
}

