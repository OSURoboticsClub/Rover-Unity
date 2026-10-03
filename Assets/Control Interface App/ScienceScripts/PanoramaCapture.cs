using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Newtonsoft.Json.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngineDebug = UnityEngine.Debug;

public class PanoramaCapture : MonoBehaviour
{
    [Header("Camera and UI")]
    public RawImage cameraImage;
    public Image crosshairOverlay;
    public TextMeshProUGUI headingOverlay;
    public Button captureButton;

    [Header("Panorama Sweep")]
    [Tooltip("Starting yaw angle in degrees for the panorama sweep.")]
    public float startAngle = -45f;
    [Tooltip("Ending yaw angle in degrees for the panorama sweep.")]
    public float endAngle = 45f;
    [Tooltip("Yaw increment in degrees between captured frames.")]
    public float angleIncrement = 5f;
    [Tooltip("How long to wait after sending a gimbal command before capturing a frame.")]
    public float gimbalMoveDuration = 0.25f;
    [Tooltip("How long to wait after the camera settles before saving the frame.")]
    public float captureDelay = 0.1f;
    [Tooltip("Name used for the stitched panorama output.")]
    public string panoramaName = "panorama";
    [Tooltip("Output image extension used when saving frame images.")]
    public string fileExtension = "png";

    private readonly List<string> savedImagePaths = new List<string>();
    private readonly List<float> angleTargets = new List<float>();
    private bool isCapturing;
    private float latestHeading = 0f;
    private string imageDirectory;

    void Awake()
    {
        TcpMessageReceiver.imuReceived.AddListener(OnHeadingReceived);
    }

    void Start()
    {
        if (captureButton != null)
        {
            captureButton.onClick.AddListener(StartCaptureSequence);
        }

        imageDirectory = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyPictures),
            "MyGamePhotos",
            "PanoramaCapture"
        );
        Directory.CreateDirectory(imageDirectory);

        SetOverlayVisibility(false);
    }

    void OnDestroy()
    {
        TcpMessageReceiver.imuReceived.RemoveListener(OnHeadingReceived);
    }

    public void StartCaptureSequence()
    {
        if (isCapturing)
        {
            return;
        }

        if (cameraImage == null || cameraImage.texture == null)
        {
            UnityEngineDebug.LogError("PanoramaCapture requires a RawImage camera feed to be assigned.");
            return;
        }

        if (Mathf.Abs(angleIncrement) < 0.001f)
        {
            UnityEngineDebug.LogError("Angle increment must be non-zero.");
            return;
        }

        if (UdpController.inst == null)
        {
            UnityEngineDebug.LogError("UdpController is not available. The panorama capture will be unable to move the tower gimbal.");
            return;
        }

        StartCoroutine(CapturePanoramaRoutine());
    }

    IEnumerator CapturePanoramaRoutine()
    {
        isCapturing = true;
        savedImagePaths.Clear();
        angleTargets.Clear();

        BuildAngleTargets();
        string panoramaOutputName = $"{panoramaName}_{DateTime.Now:yyyyMMdd_HHmmss}";
        string captureDirectory = Path.Combine(imageDirectory, panoramaOutputName);
        Directory.CreateDirectory(captureDirectory);

        int middleIndex = Mathf.Max(0, angleTargets.Count / 2);

        for (int i = 0; i < angleTargets.Count; i++)
        {
            float targetYaw = angleTargets[i];
            float angleRadians = targetYaw * Mathf.Deg2Rad;

            SendTowerGimbalYaw(angleRadians);
            yield return new WaitForSeconds(gimbalMoveDuration);
            SendTowerGimbalYaw(0f);
            yield return new WaitForSeconds(captureDelay);

            bool isMiddleFrame = i == middleIndex;
            SetOverlayVisibility(isMiddleFrame);

            string savePath = Path.Combine(captureDirectory, $"{i:D4}.{fileExtension}");
            CaptureCurrentFrame(savePath, isMiddleFrame);
            savedImagePaths.Add(savePath);
        }

        SendTowerGimbalYaw(0f);
        SetOverlayVisibility(false);
        yield return StartCoroutine(StitchPanoramaRoutine(captureDirectory, panoramaOutputName));
        isCapturing = false;
    }

    void BuildAngleTargets()
    {
        float step = Mathf.Abs(angleIncrement);
        float direction = Mathf.Sign(endAngle - startAngle);
        if (direction == 0f)
        {
            direction = 1f;
        }

        angleTargets.Add(startAngle);
        float current = startAngle;
        while (Mathf.Abs(current - endAngle) > 0.0001f)
        {
            current += direction * step;
            if ((direction > 0f && current > endAngle) || (direction < 0f && current < endAngle))
            {
                current = endAngle;
            }

            angleTargets.Add(current);
            if (Mathf.Abs(current - endAngle) < 0.0001f)
            {
                break;
            }
        }
    }

    void CaptureCurrentFrame(string filePath, bool overlayCrosshair)
    {
        Texture2D sourceTexture = cameraImage.texture as Texture2D;
        if (sourceTexture == null)
        {
            UnityEngineDebug.LogWarning("The current camera Image texture is not a Texture2D. The panorama frame could not be saved.");
            return;
        }

        Texture2D savedTexture = FlipTextureVertically(sourceTexture);

        if (overlayCrosshair)
        {
            DrawCrosshair(savedTexture);
            DrawHeadingText(savedTexture, GetHeadingLabel());
        }

        byte[] bytes = savedTexture.EncodeToPNG();
        File.WriteAllBytes(filePath, bytes);
        Destroy(savedTexture);
        UnityEngineDebug.Log($"Saved panorama frame to {filePath}");
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

    string GetHeadingLabel()
    {
        return float.IsNaN(latestHeading) ? "N/A" : latestHeading.ToString("F1");
    }

    void DrawCrosshair(Texture2D texture)
    {
        int centerX = texture.width / 2;
        int centerY = texture.height / 2;
        int size = Mathf.Max(4, Mathf.RoundToInt(Mathf.Min(texture.width, texture.height) * 0.03f));
        Color crosshairColor = new Color(0f, 1f, 0f, 1f);

        for (int x = centerX - size; x <= centerX + size; x++)
        {
            texture.SetPixel(x, centerY, crosshairColor);
        }

        for (int y = centerY - size; y <= centerY + size; y++)
        {
            texture.SetPixel(centerX, y, crosshairColor);
        }

        texture.Apply();
    }

    void DrawHeadingText(Texture2D texture, string headingText)
    {
        int centerX = texture.width / 2;
        int centerY = texture.height / 2;
        int startX = centerX - (headingText.Length * 4);
        int startY = centerY + 20;
        Color textColor = new Color(0f, 1f, 0f, 1f);

        DrawPixelText(texture, headingText, startX, startY, textColor, 1);
    }

    void DrawPixelText(Texture2D texture, string text, int startX, int startY, Color color, int scale)
    {
        int cursorX = startX;
        for (int i = 0; i < text.Length; i++)
        {
            char ch = text[i];
            string[] glyph = GetGlyph(ch);
            if (glyph == null)
            {
                cursorX += 6 * scale;
                continue;
            }

            for (int row = glyph.Length - 1; row >= 0; row--)
            {
                for (int col = 0; col < glyph[row].Length; col++)
                {
                    if (glyph[row][col] == '#')
                    {
                        int pixelX = cursorX + (col * scale);
                        int pixelY = startY + ((glyph.Length - 1 - row) * scale);
                        for (int sy = 0; sy < scale; sy++)
                        {
                            for (int sx = 0; sx < scale; sx++)
                            {
                                int drawX = pixelX + sx;
                                int drawY = pixelY + sy;
                                if (drawX >= 0 && drawX < texture.width && drawY >= 0 && drawY < texture.height)
                                {
                                    texture.SetPixel(drawX, drawY, color);
                                }
                            }
                        }
                    }
                }
            }

            cursorX += (glyph[0].Length + 1) * scale;
        }

        texture.Apply();
    }

    string[] GetGlyph(char ch)
    {
        switch (ch)
        {
            case 'H': return new[] { "#   #", "#   #", "#####", "#   #", "#   #", "#   #", "#   #" };
            case 'e': return new[] { " ##### ", "#     ", "######", "#     ", "#     ", "##### ", "      " };
            case 'a': return new[] { "  ###  ", " #   # ", "#     #", "###### #", "#     #", "#     #", "      " };
            case 'd': return new[] { "#     #", "#     #", "######", "#     #", "#     #", "#     #", "      " };
            case 'i': return new[] { " ### ", "  #  ", "  #  ", "  #  ", "  #  ", " ### ", "     " };
            case 'n': return new[] { "#    #", "##   #", "# #  #", "#  # #", "#   ##", "#    #", "      " };
            case 'g': return new[] { " ##### ", "#     #", "#     #", "#  ###", "#   # ", " ###  ", "      " };
            case ':': return new[] { "  ", " ## ", " ## ", "  ", " ## ", " ## ", "  " };
            case ' ': return new[] { "      ", "      ", "      ", "      ", "      ", "      ", "      " };
            case '0': return new[] { " ### ", "#   #", "#   #", "#   #", "#   #", " ### ", "     " };
            case '1': return new[] { "  #  ", " ##  ", "  #  ", "  #  ", "  #  ", " ### ", "     " };
            case '2': return new[] { " ### ", "#   #", "    #", "   # ", "  #  ", "#####", "     " };
            case '3': return new[] { "###  ", "   # ", "  ## ", "   # ", "#   #", " ### ", "     " };
            case '4': return new[] { "  #  ", " # # ", "#   #", "#####", "   # ", "   # ", "     " };
            case '5': return new[] { "#####", "#    ", "#### ", "    #", "#   #", " ### ", "     " };
            case '6': return new[] { " ### ", "#    ", "#####", "#   #", "#   #", " ### ", "     " };
            case '7': return new[] { "#####", "    #", "   # ", "  #  ", " #   ", "#    ", "     " };
            case '8': return new[] { " ### ", "#   #", " ### ", "#   #", "#   #", " ### ", "     " };
            case '9': return new[] { " ### ", "#   #", "#   #", " ####", "    #", " ### ", "     " };
            case '.': return new[] { "   ", "   ", "   ", "   ", " ##", " ##", "   " };
            case '-': return new[] { "   ", "   ", "###", "   ", "   ", "   ", "   " };
            case '/': return new[] { "   #", "  # ", " #  ", "#   ", "   ", "   ", "   " };
            default: return null;
        }
    }

    void SetOverlayVisibility(bool visible)
    {
        if (crosshairOverlay != null)
        {
            crosshairOverlay.gameObject.SetActive(visible);
        }
    }

    void SendTowerGimbalYaw(float yawValueRadians)
    {
        JObject msg = new JObject
        {
            ["topic"] = "tower_gimbal/control",
            ["msgType"] = "rover2_control_interface/msg/OdrivePanTiltControlMessage",
            ["data"] = new JObject
            {
                ["go_home"] = false,
                ["is_angle"] = true,
                ["stabalize"] = false,
                ["pitch"] = 0.0,
                ["yaw"] = 0.0,
                ["roll"] = yawValueRadians,
            }
        };

        UdpController.inst.PublishMessage(msg.ToString());
    }

    void OnHeadingReceived(string message)
    {
        string[] parts = message.Split(';');
        if (parts.Length < 2)
        {
            return;
        }

        if (float.TryParse(parts[1], out float heading))
        {
            latestHeading = heading;
        }
    }

    IEnumerator StitchPanoramaRoutine(string captureDirectory, string outputBaseName)
    {
        string scriptPath = Path.Combine(
            Application.dataPath,
            "Control Interface App",
            "ScienceScripts",
            "stitch_images.py"
        );

        if (!File.Exists(scriptPath))
        {
            UnityEngineDebug.LogError($"Unable to locate panorama stitch script at '{scriptPath}'.");
            yield break;
        }

        ProcessStartInfo startInfo = new ProcessStartInfo
        {
            FileName = "python3",
            Arguments = $"\"{scriptPath}\" \"{captureDirectory}\" \"{outputBaseName}\" \"{fileExtension}\"",
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true
        };

        Process process = null;
        try
        {
            process = Process.Start(startInfo);
        }
        catch (Exception e)
        {
            UnityEngineDebug.LogError($"Unable to run panorama stitch step: {e.Message}");
            yield break;
        }

        while (!process.HasExited)
        {
            yield return null;
        }

        string stdout = process.StandardOutput.ReadToEnd();
        string stderr = process.StandardError.ReadToEnd();

        if (process.ExitCode != 0)
        {
            UnityEngineDebug.LogError($"Panorama stitching failed. stderr: {stderr}");
            yield break;
        }

        UnityEngineDebug.Log(stdout);
        string outputPath = Path.Combine(captureDirectory, $"{outputBaseName}.{fileExtension}");
        if (File.Exists(outputPath))
        {
            UnityEngineDebug.Log($"Panorama saved to {outputPath}");
        }
    }
}
