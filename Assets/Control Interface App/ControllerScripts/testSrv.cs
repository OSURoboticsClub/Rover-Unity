using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json.Linq;
using System.Threading;
using System.Diagnostics;

public class TestSender : MonoBehaviour
{
    // Start is called before the first frame update

    [SerializeField] private TextAsset testServiceJson;

    private JObject testService;

    private JObject response;
    async void Start()
    {
        StartCoroutine(RunAtInterval());
        testService = JObject.Parse(testServiceJson.text);

        testService["service"]="test_srv";
        testService["request"]["data"]=true;
        string msg = testService.ToString();
        response = await UdpController.inst.PublishClientReq(msg);

        JObject testAction = JObject.Parse("{\"action\": \"fibonacci\", \"actionType\": \"action_tutorials_interfaces/action/Fibonacci\", \"request\": {\"order\": 4}}");
        UdpController.inst.PublishActionReq(testAction.ToString());
    }

    void Update()
    {
        JObject msg = UdpController.inst.GetLatestActionFeedback("fibonacci");
        if(msg != null)
        {
            UnityEngine.Debug.Log(msg.ToString());
        }
    }
    
    IEnumerator RunAtInterval()
    {
        
        while (true) // Or some condition
        {
            
            yield return new WaitForSeconds(1f); // Wait 1 second
        }
        
    }
}