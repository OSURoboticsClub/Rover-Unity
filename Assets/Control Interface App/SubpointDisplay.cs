using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;

public class SubpointDisplay : MonoBehaviour
{
    public static SubpointDisplay inst;
    [SerializeField] GameObject subpoint;
    [SerializeField] Transform iconParent;

    [SerializeField] string fullMsg = "";

    void Awake()
    {
        inst = this;
    }

    public void Receive(string msg){
        string[] parts = msg.Split(";");
        fullMsg += parts[^1];

        List<Root> myDeserializedClass = JsonConvert.DeserializeObject<List<Root>>(fullMsg);
        foreach(var x in myDeserializedClass){
            Vector2 pos = MapController.instance.GetWorldPosition(x.latitude, x.longitude);
            var obj = GameObject.Instantiate(subpoint, pos, Quaternion.identity);
            obj.transform.SetParent(iconParent);
        }
    }
}

public class Root
{
    public double latitude { get; set; }
    public double longitude { get; set; }
}