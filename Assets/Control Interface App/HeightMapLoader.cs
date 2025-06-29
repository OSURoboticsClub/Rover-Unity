using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEditor;

public class HeightMapLoader : MonoBehaviour
{
    public string filePath = "Assets/heightmap.csv"; // Path to your CSV height map
    public float heightScale = 50f; // Adjust this based on actual elevation changes
    public float terrainScale = 10f; // Scale factor to increase terrain size
    public Terrain terrain; // Assign in the Unity Editor
    public Texture2D desertTexture; // Assign your TIF file in Unity Editor
    public float pixelsPerUnit = 1f; // Adjust this based on how many pixels per unit you want
    
    // Render distance settings
    public float viewDistance = 10000f; // Maximum distance at which the terrain will be visible
    public int basemapDistance = 5000; // Distance used for the lowest detail level
    
    // Height Gradient properties
    public bool useHeightGradient = true; // Toggle to use height gradient
    public Color lowHeightColor = new Color(0, 0, 1); // Blue for low areas
    public Color midHeightColor = new Color(0, 1, 0); // Green for middle areas
    public Color highHeightColor = new Color(1, 0, 0); // Red for high areas
    public float minGradientHeight = 0f; // Minimum height for gradient
    public float maxGradientHeight = 50f; // Maximum height for gradient

    private int width;
    private int height;
    private float[,] heightMap;
    private float actualMinHeight;
    private float actualMaxHeight;

    public void Start()
    {
        heightMap = ReadCSV(filePath);
        GenerateTerrain(heightMap);
        ClearTerrainLayers();
        
        if (useHeightGradient) {
            ApplyHeightGradient();
        } else {
            ApplyDesertTexture();
        }
        
        SetRenderDistance();
        CalculateRequiredTextureSize();
    }

    float[,] ReadCSV(string path)
    {
        string[] lines = File.ReadAllLines(path);
        height = lines.Length;
        width = lines[0].Split(',').Length;

        float[,] map = new float[width, height];

        for (int y = 0; y < height; y++)
        {
            string[] values = lines[y].Split(',');
            for (int x = 0; x < width; x++)
            {
                map[x, y] = float.Parse(values[x]);
            }
        }

        return map;
    }

    void GenerateTerrain(float[,] heightMap)
    {
        TerrainData terrainData = new TerrainData();
        
        // Fix for rectangular heightmaps - use separate resolutions for width and height
        int heightmapResolutionWidth = Mathf.NextPowerOfTwo(width) + 1;
        int heightmapResolutionHeight = Mathf.NextPowerOfTwo(height) + 1;
        int heightmapResolution = Mathf.Max(heightmapResolutionWidth, heightmapResolutionHeight);
        
        terrainData.heightmapResolution = heightmapResolution;
        
        // Apply terrain scale to increase the overall size
        terrainData.size = new Vector3(width * terrainScale, heightScale, height * terrainScale);

        // Normalize height values
        actualMinHeight = float.MaxValue;
        actualMaxHeight = float.MinValue;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (heightMap[x, y] < actualMinHeight) actualMinHeight = heightMap[x, y];
                if (heightMap[x, y] > actualMaxHeight) actualMaxHeight = heightMap[x, y];
            }
        }

        float range = actualMaxHeight - actualMinHeight;
        
        // Create a properly sized normalized heightmap that matches terrain resolution
        float[,] normalizedHeights = new float[heightmapResolution - 1, heightmapResolution - 1];
        
        // Fill normalized heights with interpolated values
        for (int y = 0; y < heightmapResolution - 1; y++)
        {
            for (int x = 0; x < heightmapResolution - 1; x++)
            {
                // Calculate the corresponding position in our original heightmap
                int sourceX = Mathf.Min(Mathf.FloorToInt((float)x / (heightmapResolution - 1) * width), width - 1);
                int sourceY = Mathf.Min(Mathf.FloorToInt((float)y / (heightmapResolution - 1) * height), height - 1);
                
                // Normalize the height value
                normalizedHeights[y, x] = (heightMap[sourceX, sourceY] - actualMinHeight) / range;
            }
        }

        terrainData.SetHeights(0, 0, normalizedHeights);

        // Assign the terrain data to the existing terrain in the scene
        terrain.terrainData = terrainData;
        
        // Log the terrain size and height range for debugging
        Debug.Log($"Terrain dimensions: {terrainData.size.x} x {terrainData.size.y} x {terrainData.size.z}");
        Debug.Log($"Actual height range: {actualMinHeight} to {actualMaxHeight}");
        
        // Update min/max gradient heights based on actual terrain if not set manually
        if (minGradientHeight == 0 && maxGradientHeight == 50) {
            minGradientHeight = 0;
            maxGradientHeight = heightScale;
            Debug.Log($"Setting gradient height range: {minGradientHeight} to {maxGradientHeight}");
        }
    }

    void ApplyHeightGradient()
    {
        // Create a new material with the height gradient shader
        Shader heightGradientShader = Shader.Find("Custom/TerrainHeightGradient");
        
        if (heightGradientShader != null)
        {
            Material gradientMaterial = new Material(heightGradientShader);
            
            // Set the shader properties
            gradientMaterial.SetFloat("_MinHeight", minGradientHeight);
            gradientMaterial.SetFloat("_MaxHeight", maxGradientHeight);
            gradientMaterial.SetColor("_LowColor", lowHeightColor);
            gradientMaterial.SetColor("_MidColor", midHeightColor);
            gradientMaterial.SetColor("_HighColor", highHeightColor);
            
            // Set the desert texture if available
            if (desertTexture != null)
            {
                gradientMaterial.SetTexture("_MainTex", desertTexture);
            }
            
            // Apply the material to the terrain
            terrain.materialTemplate = gradientMaterial;
            
            Debug.Log("Applied height gradient shader to terrain");
        }
        else
        {
            Debug.LogError("Height gradient shader not found! Make sure to create the shader 'Custom/TerrainHeightGradient'");
            // Fall back to regular texture
            ApplyDesertTexture();
        }
    }

    void SetRenderDistance()
    {
        if (terrain != null)
        {
            // Set render distance parameters - only for terrain itself
            terrain.drawHeightmap = true;
            terrain.drawTreesAndFoliage = false; // Disable trees and foliage
            
            // Increase view distance on the terrain
            terrain.heightmapPixelError = 5; // Lower for better quality, higher for better performance
            terrain.basemapDistance = basemapDistance;
            
            // Disable detail and tree rendering settings
            terrain.detailObjectDistance = 0;
            terrain.treeDistance = 0;
            terrain.treeBillboardDistance = 0;
            
            // Set camera's far clip plane to match our view distance
            if (Camera.main != null)
            {
                Camera.main.farClipPlane = viewDistance;
                Debug.Log($"Camera far clip plane set to {viewDistance} units");
            }
            
            // For Unity 2019.1 and newer, use these settings
            #if UNITY_2019_1_OR_NEWER
            terrain.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.TwoSided;
            #endif
            
            Debug.Log($"Terrain render distance set to {viewDistance} units");
        }
        else
        {
            Debug.LogError("Terrain reference is not set. Please assign a terrain in the inspector.");
        }
    }

    void ClearTerrainLayers()
    {
        if (terrain != null && terrain.terrainData != null)
        {
            terrain.terrainData.terrainLayers = new TerrainLayer[0]; // Clears all applied layers
        }
    }

    void ApplyDesertTexture()
    {
        if (desertTexture == null)
        {
            Debug.LogError("Desert texture not assigned! Please assign it in the Unity Inspector.");
            return;
        }

        // Get the terrain's dimensions in world units
        float terrainWidth = terrain.terrainData.size.x;
        float terrainHeight = terrain.terrainData.size.z;

        // Create a new TerrainLayer
        TerrainLayer desertLayer = new TerrainLayer();
        desertLayer.diffuseTexture = desertTexture; // Use assigned desert texture
        desertLayer.tileSize = new Vector2(terrainWidth, terrainHeight); // Set tile size to match terrain dimensions exactly
        
        // Apply the desert texture to the terrain
        terrain.terrainData.terrainLayers = new TerrainLayer[] { desertLayer };
        
        Debug.Log($"Applied texture to terrain: {terrainWidth} x {terrainHeight} units");
    }

    void CalculateRequiredTextureSize()
    {
        // Get the terrain's dimensions (width and height) in Unity world units
        float terrainWidth = terrain.terrainData.size.x;
        float terrainHeight = terrain.terrainData.size.z;

        // Calculate the required texture resolution (in pixels)
        int textureWidth = Mathf.CeilToInt(terrainWidth * pixelsPerUnit);
        int textureHeight = Mathf.CeilToInt(terrainHeight * pixelsPerUnit);

        // Print the required texture size for tiling the terrain once
       
    }
}


