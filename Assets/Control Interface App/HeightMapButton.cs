using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

// puts a button on the Height Map Loader script in the inspector, so we can load it before runtime
<<<<<<< HEAD
#if UNITY_EDITOR
=======
#if UnityEditor
>>>>>>> abd9d44511022284dfe8ab0960d80129c4b05846
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
<<<<<<< HEAD
#endif
=======
#endif 
>>>>>>> abd9d44511022284dfe8ab0960d80129c4b05846
