using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TaranisEnabler : MonoBehaviour
{
    // the commands from autonomy or x-box controller automatically disable the taranis for now
    // so this button is only to enable it again
    
    private void Start() {
        string message = $"taranisenable;true"; // true or false doesnt actually do anything
        TcpController.inst.Publish(message);
    }

    public void OnClick() {
        string message = $"taranisenable;true";
        TcpController.inst.Publish(message);
    }
}
