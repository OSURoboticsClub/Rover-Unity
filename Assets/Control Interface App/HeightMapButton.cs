using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

// puts a button on the Height Map Loader script in the inspector, so we can load it before runtime
#if UnityEditor
[CustomEditor(typeof(HeightMapLoader))]
public class HeightMapButton : Editor {
    public override void OnInspectorGUI() {
        DrawDefaultInspector();

        HeightMapLoader script = (HeightMapLoader)target;
        if (GUILayout.Button("Load map")) {
            script.Start();
        }
    }
}
#endif 