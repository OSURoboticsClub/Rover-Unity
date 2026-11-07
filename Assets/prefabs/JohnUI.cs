using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;

public class JohnUI : MonoBehaviour
{
    private List<Button> buttons = new List<Button>();

    private VisualElement contentSlot;

    [Header("Main UI Layout")]
    public UIDocument topDocument; // Assign your Top UI Document component here

    [Header("Name of button, and page it links to")]
    public List<string> buttonNames = new List<string>();

    public List<GUIHandlerBase> GUIHandlers = new List<GUIHandlerBase>();
    private bool hasEnabled = false;
    

    void OnEnable()
    {
        Debug.Log("WTFFF WHY ARE YOU RUNNNI NG " + hasEnabled);
        if(hasEnabled) return;
        hasEnabled = true;

        // 1. Get the root element of your top UI document
        VisualElement root = topDocument.rootVisualElement;

        // 2. Find the empty content slot we created in UI Builder
        contentSlot = root.Q<VisualElement>("ui-content-slot");

        if(buttonNames.Count != GUIHandlers.Count)
        {
            Debug.LogError("Button Names count and GUIHandlers count missmatch");
            return;
        }
        for ( int i = 0; i < buttonNames.Count; i++)
        {
            Debug.Log(i);
            Debug.Log(buttonNames);
            Button btn = root.Q<GroupBox>("ButtonsTop").Q<Button>(buttonNames[i]);
            GUIHandlerBase handler = GUIHandlers[i];
            if (btn != null) btn.clicked += () => SwitchPage(handler);

            
        }
        // Optional: Load the first page by default on start
        if (GUIHandlers[0] != null) { 
            SwitchPage(GUIHandlers[0]);
            root.Q<GroupBox>("ButtonsTop").Q<Button>(buttonNames[0]).Focus();
        }
    }

    private void SwitchPage(GUIHandlerBase guiHandler)
    {
        print("Yippeee");
        if (contentSlot == null || guiHandler == null) return;

        // Clear the current bottom UI document content out of the slot
        contentSlot.Clear();

        // Instantiate the new .uxml asset into a new VisualElement
        VisualElement newPageInstance = guiHandler.OpenPage();

        // Ensure the newly instantiated page expands to fill the container layout
        newPageInstance.style.flexGrow = 1;

        // Add it directly into the layout flow beneath your top UI
        contentSlot.Add(newPageInstance);
    }
}