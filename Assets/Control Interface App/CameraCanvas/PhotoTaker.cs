using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System;

public class PhotoTaker : MonoBehaviour
{
    public RawImage cameraImage;
    public string deviceDirectory;
    private string filePath;
    private int count = 0;

    private void Start() {
        // Get OS Pictures folder
        string picturesPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyPictures),
            "MyGamePhotos"
        );

        // Make sure main folder exists
        Directory.CreateDirectory(picturesPath);

        // Add device-specific subfolder
        filePath = Path.Combine(picturesPath, deviceDirectory);
        Directory.CreateDirectory(filePath);

        count = DetermineStartingCount();
        Debug.Log($"Saving photos to: {filePath}");
    }

    int DetermineStartingCount() {
        // Checks for existing files to determine starting index
        for (int numToCheck = 0; numToCheck < 100; numToCheck++) {
            string fileNameToCheck = Path.Combine(filePath, $"{numToCheck}.png");
            if (!File.Exists(fileNameToCheck)) return numToCheck;
        }
        return 0;
    }

    public void SaveTexture()
    {
        Texture2D image = cameraImage.texture as Texture2D;
        if (image == null) {
            Debug.LogError("No texture found on cameraImage!");
            return;
        }

        byte[] bytes = image.EncodeToPNG();
        string savePath = Path.Combine(filePath, $"{count}.png");

        File.WriteAllBytes(savePath, bytes);
        Debug.Log($"Image saved as {savePath}");
        count++;
    }
}
