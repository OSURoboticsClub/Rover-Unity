using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WarningsPanelController : MonoBehaviour
{
    [SerializeField] Transform parent;
    [SerializeField] List<bool> warnings = new();
    [SerializeField] GameObject disconnectedWarning;
    [SerializeField] GameObject noCam1Warning;
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
        }
        if (noCam2Feed != warnings[2])
        {
            warnings[2] = noCam2Feed;
            noCam2Warning.SetActive(noCam2Feed);
        }
    }
}
