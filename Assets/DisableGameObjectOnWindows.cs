using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisableGameObjectOnWindows : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        if (Application.platform == RuntimePlatform.WindowsPlayer ||
            Application.platform == RuntimePlatform.WindowsEditor) {
            gameObject.SetActive(false);
        }
    }
}
