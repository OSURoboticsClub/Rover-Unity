using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEditor;
using System.Threading;

public class MapLoader : MonoBehaviour
{
    public static MapLoader instance;
    [SerializeField] Transform activeMap;
    [SerializeField] GameObject mapTile;
    string folderPath = @"C:\Users\matt\Documents\GitHub\Rover-Unity\Assets\Control Interface App\Maps\Merryfield";

    private void Awake() {
        instance = this;
    }

    public void Run() {
        Start();
    }


    private void Start() {
#if UNITY_EDITOR
        for (int i = activeMap.childCount - 1; i >= 0; i--) {
            Transform child = activeMap.GetChild(i);
            DestroyImmediate(child.gameObject);
        }
#endif

            string[] files = Directory.GetFiles(folderPath);
        List<string> filteredFiles = new();
        foreach (var x in files) {
            if (x.Contains(".meta")) continue;
            if (x.Contains(".txt")) continue;
            string[] split = x.Split("Assets");
            filteredFiles.Add("Assets" + split[1]);
        }

        int fileCount = (int)Mathf.Sqrt(filteredFiles.Count);
        Debug.Log("Map is " + fileCount + "x" + fileCount);

        foreach(var x in filteredFiles) {
            Debug.Log(x);
        }

        for (int x = 0; x < fileCount; x++) {
            for (int y = 0; y < fileCount; y++) {
                int spriteIndex = y + x * fileCount;
                Debug.Log(spriteIndex);
                Sprite sprite = AssetDatabase.LoadAssetAtPath<Sprite>(filteredFiles[spriteIndex]);
                Vector3 currPos = new Vector3(x, -y, 0);
                var obj = Instantiate(mapTile, activeMap);
                obj.transform.position = currPos;
                obj.GetComponent<SpriteRenderer>().sprite = sprite;
            }
        }
    }
}

[CustomEditor(typeof(MapLoader))]
public class MyComponentEditor : Editor {
    public override void OnInspectorGUI() {
        DrawDefaultInspector();

        MapLoader myTarget = (MapLoader)target;
        if (GUILayout.Button("Import")) {
            myTarget.Run();
        }
    }
}