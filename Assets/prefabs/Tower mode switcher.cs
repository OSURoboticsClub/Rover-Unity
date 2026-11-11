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
    [SerializeField] private TextAsset antennaControlServiceJson;
    [SerializeField] private List<Button> buttons;
    private async Task AntennaControlServiceCall(int mode)
    {
        JObject msg = JObject.Parse(antennaControlServiceJson.text);

        msg["service"]="/gs_tower_control/service";
        msg["request"]["mode"] = (int)mode;

        await UdpController.inst.PublishClientReq(msg.ToString(), "/gs_tower_control/service");
    }
    public void switch_tower_mode(int mode)
    {
        _ = AntennaControlServiceCall(mode);
        
    }
    public void Update()
    {
        
    }
}
