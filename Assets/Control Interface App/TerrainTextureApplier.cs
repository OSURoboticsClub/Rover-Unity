using UnityEngine;

public class TerrainTextureApplier : MonoBehaviour
{
    public Texture2D satelliteTexture;  // Assign your satellite image in the inspector

    void Start()
    {
        // Get the terrain component
        Terrain terrain = GetComponent<Terrain>();

        // Create a new TerrainLayer and assign the satellite texture
        TerrainLayer terrainLayer = new TerrainLayer();
        terrainLayer.diffuseTexture = satelliteTexture;

        // Set the terrain layers
        terrain.terrainData.terrainLayers = new TerrainLayer[] { terrainLayer };

        // Adjust the tiling of the texture based on terrain size (optional)
        float terrainWidth = terrain.terrainData.size.x;
        float terrainHeight = terrain.terrainData.size.z;

        // Tiling (adjust these values based on your image and terrain size)
        terrainLayer.tileSize = new Vector2(terrainWidth / satelliteTexture.width, terrainHeight / satelliteTexture.height);
    }
}

