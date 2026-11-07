using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class DriveGUIHandler : GUIHandlerBase
{
    [Header("Camera Feed UI Document")]
    [SerializeField] public VisualTreeAsset feed;
    private List<VisualElement> feeds = new List<VisualElement>();
    override public VisualElement OpenPage()
    {
        isSelected = true;
        if(pageAssetInstance == null) {
            pageAssetInstance = pageAsset.Instantiate();
            for(int i  = 0; i < 3; i++)
            {
                VisualElement newFeed = feed.Instantiate();
                feeds.Add(newFeed);
                pageAssetInstance.Q<GroupBox>("Feeds").Add(newFeed);
            }
        }
        return pageAssetInstance;
    }
    public void updateFeedAspect()
    {
        
        GroupBox feedsBox = pageAssetInstance.Q<GroupBox>("Feeds");
        float desWidthInPx = feedsBox.worldBound.width;
        Debug.Log(desWidthInPx);

        if(float.IsNaN(desWidthInPx)) {
            return;
        }
        for( int i = 0; i < feeds.Count; i++)
        {
            feeds[i].style.width = desWidthInPx/3;
            feeds[i].style.height = (desWidthInPx/3) * 0.75f;
        }
    }

    public void Update()
    {
        if (pageAssetInstance == null) return;
        updateFeedAspect();
    }
}
