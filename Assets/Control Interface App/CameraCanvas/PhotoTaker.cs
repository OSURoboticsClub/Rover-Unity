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

        Texture2D correctedImage = FlipTextureVertically(image);
        byte[] bytes = correctedImage.EncodeToPNG();
        string savePath = Path.Combine(filePath, $"{count}.png");

        File.WriteAllBytes(savePath, bytes);
        Destroy(correctedImage);
        Debug.Log($"Image saved as {savePath}");
        count++;
    }

    Texture2D FlipTextureVertically(Texture2D source)
    {
        int width = source.width;
        int height = source.height;
        Color[] pixels = source.GetPixels();
        Color[] flippedPixels = new Color[pixels.Length];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int sourceIndex = y * width + x;
                int flippedIndex = (height - 1 - y) * width + x;
                flippedPixels[flippedIndex] = pixels[sourceIndex];
            }
        }

        Texture2D flippedTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
        flippedTexture.SetPixels(flippedPixels);
        flippedTexture.Apply();
        return flippedTexture;
    }
}
