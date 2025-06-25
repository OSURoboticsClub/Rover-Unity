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
    // Start is called before the first frame update
    public void SaveTexture()
    {
    
      Texture2D image = cameraImage.texture as Texture2D;
      byte[] bytes = image.EncodeToPNG();
      File.WriteAllBytes(filePath + deviceDirectory +"/"+ count.ToString() + ".png",bytes);
      Debug.Log("Image saved");
      count+=1;
    }
}
