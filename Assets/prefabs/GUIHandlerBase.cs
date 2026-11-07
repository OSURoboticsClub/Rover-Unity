using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEngine;
using UnityEngine.UIElements;

public class GUIHandlerBase : MonoBehaviour
{
    [Header("UI Document for this page")]
    [SerializeField] public VisualTreeAsset pageAsset;
    public bool isSelected = false;
    public VisualElement pageAssetInstance = null;
    virtual public VisualElement OpenPage()
    {
        isSelected = true;
        if(pageAssetInstance == null) pageAssetInstance = pageAsset.Instantiate();
        return pageAssetInstance;

    }
    
}
