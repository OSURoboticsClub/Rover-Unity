using System;
using UnityEngine;
using sensor_msgs.msg;
using System.Collections.Concurrent;

namespace ROS2
{
    public class PointCloudHandler : MonoBehaviour
    {
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

        private byte[] byteArray;
        private bool isMessageReceived = false;
        private int size;

        private Vector3[] pcl;
        private Color[] pcl_color;

        private int width;
        private int height;
        private int row_step;
        private int point_step;

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
            ros2Node = ros2UnityComponent.CreateNode("PointCloudHandlerNode");
	    
            // Create the subscription to the PointCloud2 topic
            pointCloudSubscription = ros2Node.CreateSubscription<PointCloud2>(
                "/camera/camera/depth/color/points", msg => ReceivePointCloudMessage(msg));
        }

        private void ReceivePointCloudMessage(PointCloud2 message)
        {
            // Parse the PointCloud2 message and extract data
            size = message.Data.Length;

            byteArray = new byte[size];
            byteArray = message.Data;
	   
            width = (int)message.Width;
            height = (int)message.Height;
            row_step = (int)message.Row_step;
            point_step = (int)message.Point_step;

            // Recalculate the number of points based on point_step
            size = size / point_step;

            isMessageReceived = true;
        }

        // Converts the raw byte data into Vector3 and Color arrays for the point cloud
        void ProcessPointCloudData()
        {
            pcl = new Vector3[size];
            pcl_color = new Color[size];

            int x_posi;
            int y_posi;
            int z_posi;

            float x;
            float y;
            float z;

            int rgb_posi;
            int rgb_max = 255;

            float r;
            float g;
            float b;

            for (int n = 0; n < size; n++)
            {
                // Extract 3D coordinates
                x_posi = n * point_step + 0;
                y_posi = n * point_step + 4;
                z_posi = n * point_step + 8;

                x = BitConverter.ToSingle(byteArray, x_posi);
                y = BitConverter.ToSingle(byteArray, y_posi);
                z = BitConverter.ToSingle(byteArray, z_posi);

                // Extract RGB color values
                rgb_posi = n * point_step + 16;

                b = byteArray[rgb_posi + 0];
                g = byteArray[rgb_posi + 1];
                r = byteArray[rgb_posi + 2];

                // Normalize RGB values to [0,1]
                r = r / rgb_max;
                g = g / rgb_max;
                b = b / rgb_max;

                // Store position and color
                pcl[n] = new Vector3(x, z, y); // Flip Y/Z axis to match Unity's coordinate system
                pcl_color[n] = new Color(r, g, b);
            }
        }

        public Vector3[] GetPCL()
        {
            if (isMessageReceived)
            {
                ProcessPointCloudData();
                isMessageReceived = false;
            }
            return pcl;
        }

        public Color[] GetPCLColor()
        {
            return pcl_color;
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
            	Debug.Log("Har");
                meshRenderer.material.SetFloat("_PointSize", pointSize);
                pointCloudQueue.TryDequeue(out var pointCloudData);
                UpdateMesh(pointCloudData.Item1, pointCloudData.Item2);
            }

            // Process incoming point cloud data on the main thread
            
        }
    }
}

