using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MapOpacitySlider : MonoBehaviour
{
    [SerializeField] SpriteRenderer map;
    [SerializeField] Slider slider;

    private void Start()
    {
        SliderChanged();
    }

    public void SliderChanged()
    {
        map.color = new Color(map.color.r, map.color.g, map.color.b, slider.value);
    }
}
