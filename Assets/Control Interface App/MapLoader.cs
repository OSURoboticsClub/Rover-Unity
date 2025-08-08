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
    [SerializeField] string folderPath = @"C:\Users\matt\Documents\GitHub\Rover-Unity\Assets\Control Interface App\Maps\Drumheller";

    private void Awake() {
        instance = this;
    }

    public void Run() {
        for (int i = activeMap.childCount - 1; i >= 0; i--) {
            Transform child = activeMap.GetChild(i);
            DestroyImmediate(child.gameObject);
        }

        string[] files = Directory.GetFiles(folderPath);
        Debug.Log(folderPath);
        List<string> filteredFiles = new();
        foreach (var x in files) {
            if (x.Contains(".meta")) continue;
            if (x.Contains(".txt")) continue;
            string[] split = x.Split("Assets");
            filteredFiles.Add("Assets" + split[1]);
        }

        int fileCount = (int)Mathf.Sqrt(filteredFiles.Count);
        Debug.Log("Map is " + fileCount + "x" + fileCount);
        foreach (var filename in filteredFiles) {
            var parts = filename.Split("~");
            var coords = parts[1].Split(",");
            //Debug.Log(coords[0] + ", " + coords[1]);
            int x = int.Parse(coords[0]);
            int y = int.Parse(coords[1]);
            Sprite sprite = AssetDatabase.LoadAssetAtPath<Sprite>(filename);
            Vector3 currPos = new Vector3(x, -y, 0);
            var obj = Instantiate(mapTile, activeMap);
            obj.transform.position = currPos;
            obj.GetComponent<SpriteRenderer>().sprite = sprite;
        }

        int half = fileCount / 2;


        for (int x = 0; x < fileCount; x++) {
            for (int y = 0; y < fileCount; y++) {
            }
        }
    }


    private void Start() { }
}
[CustomEditor(typeof(MapLoader))]

public class MyComponentEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        MapLoader myTarget = (MapLoader)target;
        if (GUILayout.Button("Import"))
        {
            myTarget.Run();
        }
    }
}


public class CustomImporter : AssetPostprocessor {
    void OnPreprocessTexture() {
        TextureImporter importer = (TextureImporter)assetImporter;
        importer.spritePixelsPerUnit = 256f; // Set your preferred default here
    }
}