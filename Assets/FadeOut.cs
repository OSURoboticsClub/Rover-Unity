using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FadeOut : MonoBehaviour
{
    SpriteRenderer spr;
    float a = 1f;
    float fadeTime = 3f;

    private void Start()
    {
        spr = GetComponent<SpriteRenderer>();
    }

    private void Update()
    {
        float delta = 1f / fadeTime * Time.deltaTime;
        a -= delta;
        if (a <= 0)
        {
            spr.enabled = false;
            Destroy(gameObject);
            return;
        }
        spr.color = new Color(spr.color.r, spr.color.g, spr.color.b, a);

    }
}
