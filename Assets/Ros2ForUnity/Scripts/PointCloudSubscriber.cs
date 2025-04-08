using System;
using UnityEngine;
using sensor_msgs.msg;

namespace ROS2
{
    public class PointCloudSubscriber : MonoBehaviour
    {
        private byte[] byteArray;
        private bool isMessageReceived = false;
        private int size;

        private Vector3[] vertices;
        private Color[] colors;
        private int[] indices;

        private int width, height, row_step, point_step;
        private ROS2UnityComponent ros2UnityComponent;
        private ROS2Node ros2Node;
        private ISubscription<PointCloud2> pointCloudSubscription;

        public int maxPoints = 300000; // Adjust based on your dataset
        private Mesh mesh;

        void Start()
        {
            ros2UnityComponent = GetComponent<ROS2UnityComponent>();
            ros2Node = ros2UnityComponent.CreateNode("PointCloudSubscriberNode");

            // Create the subscription to the PointCloud2 topic
            pointCloudSubscription = ros2Node.CreateSubscription<PointCloud2>(
                "/camera/camera/depth/color/points", ReceivePointCloudMessage);

            // Initialize Mesh
            mesh = new Mesh
            {
                indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 // Allows more than 65k vertices
            };
            GetComponent<MeshFilter>().mesh = mesh;

            // Initialize arrays for mesh data
            vertices = new Vector3[maxPoints];
            colors = new Color[maxPoints];
            indices = new int[maxPoints];
        }

        private void ReceivePointCloudMessage(PointCloud2 message)
        {
            size = message.Data.Length / (int)message.Point_step;
	    Debug.Log(message.Data.Length);
            // Clamp size to maxPoints
            if (size > maxPoints)
            {
                size = maxPoints;
            }

            byteArray = message.Data;
            width = (int)message.Width;
            height = (int)message.Height;
            row_step = (int)message.Row_step;
            point_step = (int)message.Point_step;

            isMessageReceived = true;
        }

        void Update()
        {
            if (isMessageReceived)
            {
                ProcessPointCloudData();
                isMessageReceived = false;
            }
        }

        void ProcessPointCloudData()
        {
            const float rgb_max = 255f;
            int x_posi, y_posi, z_posi;
            float x, y, z;
            int rgb_posi;
            float r, g, b;

            // Use a more direct memory access approach to speed up the loop
            for (int n = 0; n < size; n++)
            {
                x_posi = n * point_step;
                y_posi = x_posi + 4;
                z_posi = x_posi + 8;

                // Directly access byte array without using BitConverter (faster)
                x = BitConverter.ToSingle(byteArray, x_posi);
                y = BitConverter.ToSingle(byteArray, y_posi);
                z = BitConverter.ToSingle(byteArray, z_posi);

                rgb_posi = n * point_step + 16;
                b = byteArray[rgb_posi + 0] / rgb_max;
                g = byteArray[rgb_posi + 1] / rgb_max;
                r = byteArray[rgb_posi + 2] / rgb_max;

                // Store the data
                vertices[n] = new Vector3(x, z, y); // Flip Y/Z for Unity
                colors[n] = new Color(r, g, b);
                indices[n] = n; // Each vertex is its own index
            }

            // Use the mesh's existing data buffers instead of clearing every frame
            mesh.vertices = vertices;
            mesh.colors = colors;
            mesh.SetIndices(indices, MeshTopology.Points, 0);
        }
    }
}

