using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TabSelector : MonoBehaviour
{
    [SerializeField] List<Button> buttons;
    [SerializeField] List<GameObject> pages;
    [SerializeField] int currentPageIndex;

    // Start is called before the first frame update
    void Start()
    {
        foreach(Transform x in transform) {
            buttons.Add(x.GetComponent<Button>());
            // auto populate the list using the hierarchy
        }

        if(buttons.Count != pages.Count) {
            Debug.LogError("[TabSelector] number of tabs and pages don't match");
            return;
        }

        currentPageIndex = GetWhichPageIsOpenAtStart();

        for (int i = 0; i < buttons.Count; i++) {
            if(i == currentPageIndex) {
                buttons[i].GetComponent<Image>().color = new Color(51 / 255f, 51 / 255f, 51 / 255f);
                pages[i].SetActive(true);
            }
            else {
                buttons[i].GetComponent<Image>().color = new Color(29 / 255f, 29 / 255f, 29 / 255f);
                pages[i].SetActive(false);
            }
        }
    }

    int GetWhichPageIsOpenAtStart() {

        for (int i = 0; i < pages.Count; i++) {
            if (pages[i].activeSelf) return i;
        }
        return 0;
    }

    public void ClickTab(int index) {
        if (index == currentPageIndex) return;

        for (int i = 0; i < buttons.Count; i++) {
            buttons[i].GetComponent<Image>().color = new Color(29 / 255f, 29 / 255f, 29 / 255f);
            pages[i].SetActive(false);
        }

        buttons[index].GetComponent<Image>().color = new Color(51 / 255f, 51 / 255f, 51 / 255f);
        pages[index].SetActive(true);
        currentPageIndex = index;
    }
}
