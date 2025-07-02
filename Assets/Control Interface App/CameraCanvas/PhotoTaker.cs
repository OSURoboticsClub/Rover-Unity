using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;

public class PhotoTaker : MonoBehaviour
{
    public RawImage cameraImage;
    public string deviceDirectory;
    private string filePath = "Assets/Photos/";
    private int count = 0;

    private void Start() {
        count = DetermineStartingCount();
    }

    int DetermineStartingCount() {
        // checks already existing files to decide which number to start from for new pictures
        for (int numToCheck = 0; numToCheck < 100; numToCheck++) {
            string fileNameToCheck = filePath + deviceDirectory + $"/{numToCheck}.png";
            if (!File.Exists(fileNameToCheck)) return numToCheck;
        }

        return 0;
    }

    public void SaveTexture()
    {
    
      Texture2D image = cameraImage.texture as Texture2D;
      byte[] bytes = image.EncodeToPNG();
      File.WriteAllBytes(filePath + deviceDirectory +"/"+ count.ToString() + ".png",bytes);
      Debug.Log($"Image saved as {count}.png");
      count+=1;
    }
}
