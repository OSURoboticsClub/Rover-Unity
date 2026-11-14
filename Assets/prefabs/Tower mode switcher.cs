using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;



public enum TowerControlMode
{
    DISABLED = 0,
    MANUAL_CONTROL = 1,
    AUTOMATIC_CONTROL = 2,
    HOMING = 3
}
public class Towermodeswitcher : MonoBehaviour
{
    [SerializeField] private TowerSubscriber towerSubscriber;
    [SerializeField] private TextAsset antennaControlServiceJson;
    [SerializeField] private List<Button> buttons;
    [Header("Colors")]
    [SerializeField] private Color defaultColor;
    [SerializeField] private Color selectedColor;

    public void Start()
    {
        towerSubscriber.onAntennaControlStatusUpdate.AddListener(antennaControlStatusUpdate);
        for(int i = 0; i < buttons.Count; i++)
        {
            int j = i;
            buttons[i].image.color = defaultColor;
            buttons[i].onClick.AddListener(() => switch_tower_mode(j));
        }
        buttons[0].image.color = selectedColor;

    }
    private async Task AntennaControlServiceCall(int mode)
    {
        JObject msg = JObject.Parse(antennaControlServiceJson.text);

        msg["service"]="/gs_tower_control/service";
        msg["request"]["mode"] = (int)mode;

        await UdpController.inst.PublishClientReq(msg.ToString(), "/gs_tower_control/service");
    }
    public void switch_tower_mode(int mode)
    {
        if(towerSubscriber.debug)
        {
            towerSubscriber.currentStatus.operating_mode = (TowerControlMode) mode;
            Debug.Log("Tower control mode: " + mode);
            towerSubscriber.onAntennaControlStatusUpdate.Invoke(towerSubscriber.currentStatus);

        }
        else
        {
            _ = AntennaControlServiceCall(mode);
        }
    }
    public void antennaControlStatusUpdate(AntennaControlStatus newStatus)
    {
        Debug.Log("antennaControlStatusUpdate");
        for(int i = 0; i < buttons.Count; i++)
        {
            if(i == (int)newStatus.operating_mode)
            {
                buttons[i].image.color = selectedColor;
                continue;
            }
            buttons[i].image.color = defaultColor;
        }
    }
}
