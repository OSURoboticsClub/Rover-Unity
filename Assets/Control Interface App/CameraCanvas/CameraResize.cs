using UnityEngine;

public class UIResizer : MonoBehaviour
{
    public RectTransform feed1;
    public RectTransform feed2;
    public RectTransform feed3;
    
    private Vector2 defaultLeftPos = new Vector2(-640, 110);
    private Vector2 defaultMidPos = new Vector2(0, 110);
    private Vector2 defaultRighttPos = new Vector2(640, 110);

    private Vector2 mainPos = new Vector2(-293, -165);
    private Vector2 topRightPos = new Vector2(666, 134);
    private Vector2 bottomRightPos = new Vector2(666, -250);
    
    private Vector2 mainSize = new Vector2(1333, 1000);
    private Vector2 minSize = new Vector2(586, 440);
    private Vector2 defaultSize = new Vector2(640,480);
    
    public void ResetAllFeedDims(){
        feed1.anchoredPosition = defaultLeftPos;
        feed1.sizeDelta = defaultSize;
        
        feed2.anchoredPosition = defaultMidPos;
        feed2.sizeDelta = defaultSize;
               
        feed3.anchoredPosition = defaultRighttPos;
        feed3.sizeDelta = defaultSize;
    }
    
    public void MoveAndResize(RectTransform mainFeed, RectTransform topRightFeed,RectTransform bottomRightFeed)
    {
        mainFeed.anchoredPosition = mainPos;
        mainFeed.sizeDelta = mainSize;
        
        topRightFeed.anchoredPosition = topRightPos;
        topRightFeed.sizeDelta = minSize;
               
        bottomRightFeed.anchoredPosition = bottomRightPos;
        bottomRightFeed.sizeDelta = minSize;

        Debug.Log("UI element moved and resized.");
    }
    public void MaximizeFeed1(){
        MoveAndResize(feed1,feed2,feed3);
    }
    public void MaximizeFeed2(){
        MoveAndResize(feed2,feed1,feed3);
    }
    public void MaximizeFeed3(){
        MoveAndResize(feed3,feed1,feed2);
    }
}

