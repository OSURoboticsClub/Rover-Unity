using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class MessagesController : MonoBehaviour
{
    public static MessagesController inst;
    [SerializeField] string latestMsg;
    [SerializeField] TextMeshProUGUI text;

    void Awake()
    {
        inst = this;
    }

    public void DisplayMessage(string msg)
    {
        latestMsg = msg;
        Debug.Log($"Received message: {msg}");
    }

    private void Update()
    {
        text.text = latestMsg;
    }
}
