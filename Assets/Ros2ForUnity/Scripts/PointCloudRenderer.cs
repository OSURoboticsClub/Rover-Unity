using System;
using UnityEngine;
using sensor_msgs.msg;
using System.Collections.Concurrent;

namespace ROS2
{
    public class PointCloudRenderer : MonoBehaviour
    {
        public PointCloudSubscriber subscriber;
        // Thread-safe queue to store incoming point cloud data
        private ConcurrentQueue<(Vector3[], Color[])> pointCloudQueue = new ConcurrentQueue<(Vector3[], Color[])>();

        // Mesh stores the positions and colours of every point in the cloud
        private Mesh mesh;
        private MeshRenderer meshRenderer;
        private MeshFilter mf;

        // The size, positions and colours of each of the point cloud
        public float pointSize = 1f;

        [Header("MAKE SURE THESE LISTS ARE MINIMISED OR EDITOR WILL CRASH")]
        private Vector3[] positions = new Vector3[] { new Vector3(0, 0, 0), new Vector3(0, 1, 0) };
        private Color[] colours = new Color[] { new Color(1f, 0f, 0f), new Color(0f, 1f, 0f) };

        public Transform offset; // Put any gameobject that facilitates adjusting the origin of the point cloud in VR.

        private ROS2UnityComponent ros2UnityComponent;
        private ROS2Node ros2Node;
        private ISubscription<PointCloud2> pointCloudSubscription;

        void Awake()
        {
            // Ensure MeshRenderer is added to the GameObject if it's not already attached
            meshRenderer = gameObject.GetComponent<MeshRenderer>();
            if (meshRenderer == null)
            {
                meshRenderer = gameObject.AddComponent<MeshRenderer>();
            }

            // Check if the material is not set yet
            if (meshRenderer.material == null)
            {
                Shader pointCloudShader = Shader.Find("Custom/PointCloudShader");
                if (pointCloudShader != null)
                {
                    // Assign a new material with the shader if none exists
                    meshRenderer.material = new Material(pointCloudShader);
                }
                else
                {
                    Debug.LogError("Shader 'Custom/PointCloudShader' not found.");
                }
            }

            // Ensure MeshFilter is added to the GameObject if it's not already attached
            mf = gameObject.GetComponent<MeshFilter>();
            if (mf == null)
            {
                mf = gameObject.AddComponent<MeshFilter>();
            }

            // Create the mesh for rendering
            mesh = new Mesh
            {
                indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
            };

            // Set up the initial position and rotation of the GameObject based on the offset
            transform.position = offset.position;
            transform.rotation = offset.rotation;
        }

        void Start()
        {
            // Initialize ROS2 Unity components and the ROS node
            ros2UnityComponent = GetComponent<ROS2UnityComponent>();
            ros2Node = ros2UnityComponent.CreateNode("PointCloudRendererNode");

            // Create subscription to the PointCloud2 topic
            pointCloudSubscription = ros2Node.CreateSubscription<PointCloud2>(
                "/camera/camera/depth/color/points", ReceivePointCloudMessage);
        }

        private void ReceivePointCloudMessage(PointCloud2 message)
        {
            // Parse the PointCloud2 message into positions and colors
            Vector3[] newPositions = ParsePointCloudPositions(message);
            Color[] newColors = ParsePointCloudColors(message);

            // Add the new data to the thread-safe queue
            pointCloudQueue.Enqueue((newPositions, newColors));
        }

        private Vector3[] ParsePointCloudPositions(PointCloud2 message)
        {
            // Implement logic to parse PointCloud2 message into Vector3 positions
            // This is just a placeholder; replace with your actual parsing logic
            return new Vector3[] { new Vector3(0, 0, 0), new Vector3(1, 1, 1) };
        }

        private Color[] ParsePointCloudColors(PointCloud2 message)
        {
            // Implement logic to parse PointCloud2 message into Color array
            // This is just a placeholder; replace with your actual parsing logic
            return new Color[] { new Color(1f, 0f, 0f), new Color(0f, 1f, 0f) };
        }

        private void UpdateMesh(Vector3[] newPositions, Color[] newColors)
        {
            if (newPositions == null || newColors == null || newPositions.Length == 0)
            {
                return;
            }

            // Clear the mesh and update vertices and colors
            mesh.Clear();
            mesh.vertices = newPositions;
            mesh.colors = newColors;

            int[] indices = new int[newPositions.Length];
            for (int i = 0; i < newPositions.Length; i++)
            {
                indices[i] = i;
            }

            mesh.SetIndices(indices, MeshTopology.Points, 0);

            // Ensure MeshFilter is valid before setting the mesh
            if (mf != null)
            {
                mf.mesh = mesh;
            }
            else
            {
                Debug.LogError("MeshFilter is not assigned properly.");
            }
        }

        void Update()
        {
            // Set the position and rotation of the mesh based on the offset transform
            transform.position = offset.position;
            transform.rotation = offset.rotation;

            // Set the point size in the shader
            if (meshRenderer != null && meshRenderer.material != null)
            {
                meshRenderer.material.SetFloat("_PointSize", pointSize);
            }

            // Process incoming point cloud data on the main thread
            if (pointCloudQueue.TryDequeue(out var pointCloudData))
            {
                UpdateMesh(pointCloudData.Item1, pointCloudData.Item2);
            }
        }
    }
}

