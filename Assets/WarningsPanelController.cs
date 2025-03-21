using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class WarningsPanelController : MonoBehaviour
{
    [SerializeField] Transform parent;
    [SerializeField] List<bool> warnings = new();
    [SerializeField] GameObject disconnectedWarning;
    [SerializeField] GameObject noCam1Warning;
    [SerializeField] TextMeshProUGUI cam1Text;
    [SerializeField] GameObject noCam2Warning;

    private void Update()
    {
        bool isDisconnected = TcpController.inst.disconnected;
        if(isDisconnected != warnings[0])
        {
            warnings[0] = isDisconnected;
            disconnectedWarning.SetActive(isDisconnected);
        }

        bool noCam1Feed = false;
        if (streamListener.inst.timeSinceLastPacket > 2f) noCam1Feed = true;
        bool noCam2Feed = false;

        if(isDisconnected)
        {
            noCam1Feed = false;
            noCam2Feed = false;
        }


        if (noCam1Feed != warnings[1])
        {
            warnings[1] = noCam1Feed;
            noCam1Warning.SetActive(noCam1Feed);
            string timeSince = streamListener.inst.timeSinceLastPacket.ToString("F1");
            cam1Text.text = "No feed from camera 1 for " + timeSince + " seconds.";
        }
        if (noCam2Feed != warnings[2])
        {
            warnings[2] = noCam2Feed;
            noCam2Warning.SetActive(noCam2Feed);
        }
    }
}
