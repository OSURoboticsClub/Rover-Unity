using System.IO;
using UnityEngine;

public class ErrorDecoderOpener : MonoBehaviour
{
    [SerializeField] private string decoderFileName = "Snack_Run_Error_Decoder.html";

    private void Start()
    {
        //OpenDecoderHtml();
    }

    public void OpenDecoderHtml()
    {
        string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        string decoderPath = Path.Combine(projectRoot, decoderFileName);

        if (!File.Exists(decoderPath))
        {
            Debug.LogWarning($"Could not find error decoder file at: {decoderPath}");
            return;
        }

        string url = "file://" + decoderPath.Replace("\\", "/");
        Application.OpenURL(url);
    }
}
