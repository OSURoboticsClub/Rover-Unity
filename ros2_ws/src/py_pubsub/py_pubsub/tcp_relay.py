import socket
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Float32
from rover2_control_interface.msg import GPSStatusMessage
from rover2_control_interface.msg import DriveCommandMessage
from sensor_msgs.msg import Imu
from functools import partial
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist



# UDP Configuration for Image Transmission
UDP_IP = "127.0.0.1"  # Change to the Unity application's IP
UDP_PORT = 12345
PACKET_SIZE = 4096
HEADER_SIZE = 16
PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE

# Create UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class TCPServer(Node):

    def __init__(self):
        super().__init__('tcp_server_with_ros2')
        self.topic_publishers = {}  # Dictionary to store topic_name -> publisher
        self.message_type_map = {
            'set_joint_angles': Float32MultiArray
        }  # Map of topic names to message types
        self.subscribers = []
        self.tcp_client = None
        self.get_logger().info('TCP server initialized.')
        self.bridge = CvBridge()

        # Add subscriptions to multiple topics
        #self.add_subscription('imu/data', Imu)

        self.add_subscription('autonomous/auton_control_response', String)
        self.add_subscription('tower/status/gps', GPSStatusMessage)
        self.add_subscription('imu/data/heading', Float32)
        self.add_subscription('autonomous/simple_position', String)
        self.add_subscription('/joint_states', JointState)

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        self.img_subscription = self.create_subscription(
            Image,
            '/cameras/main_navigation/image_256x144',
            self.ros_img_callback,
            qos_profile
        )
        self.frame_number = 0

    def ros_img_callback(self, msg):
        self.send_image_over_udp(msg)


    def add_subscription(self, topic_name, message_type):
        # Create and store a subscription for each topic
        subscription = self.create_subscription(
            message_type,
            topic_name,
            partial(self.ros_to_tcp_callback, topic_name),
            10
        )
        self.subscribers.append(subscription)
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def send_image_over_udp(self, msg):
        """Converts ROS 2 Image message to bytes and sends it over UDP in chunks."""
        if self.frame_number >= 1000:
            self.frame_number = 0
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode image as JPEG
            _, img_encoded = cv2.imencode(".jpg", frame)
            image_bytes = img_encoded.tobytes()

            # Calculate number of packets
            num_of_packets = (len(image_bytes) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
            stream_id = 1

            # Send packets
            for i in range(num_of_packets):
                start = i * PAYLOAD_SIZE
                end = min(start + PAYLOAD_SIZE, len(image_bytes))  # Prevents reading beyond image size
                packet_data = image_bytes[start:end]

                # Construct packet: [Stream ID (4 bytes)] + [Frame Number (4 bytes)] + [Packet Index (4 bytes)] + [Total Packets (4 bytes)] + [Image Data]
                header = struct.pack("<iiii", stream_id, self.frame_number, i, num_of_packets)
                packet = header + packet_data

                # Send UDP packet
                sock.sendto(packet, (UDP_IP, UDP_PORT))

            self.get_logger().info(f"Sent frame {self.frame_number} in {num_of_packets} packets")
            self.frame_number += 1  # Increment frame count

        except Exception as e:
            self.get_logger().error(f"Failed to send image over UDP: {e}")

    def ros_to_tcp_callback(self, topic_name, msg): # Callback for messages received on ROS 2 topics
        if not(self.tcp_client):
            self.get_logger().warn("No active TCP client. Message not sent.")
            return
            
        try: # Determine message type and construct the string accordingly
            message = topic_name + ";"
            if topic_name == "tower/status/gps":
                message += f"{msg.rover_latitude};{msg.rover_longitude}"
            elif topic_name == "imu/data/heading":
                message += f"{msg.data}"
            elif topic_name == "imu/data":
                message += f"{msg.orientation.x};{msg.orientation.y};{msg.orientation.z};{msg.orientation.w}"
            elif topic_name == "/joint_states":
                for position in msg.position:
                    message += f"{position:.3f};"
                message = message[:-1]
            else:
                message += msg.data # handle string messages (not custom message type)
            
            self.tcp_client.sendall(message.encode('utf-8')) # Send the constructed string over TCP
            self.get_logger().info(f"Sent message over TCP: {message}")
        except Exception as e:
            self.get_logger().error(f"Failed to send message over TCP: {e}")

    def get_or_create_publisher(self, topic_name):
        if topic_name not in self.topic_publishers: # If the publisher doesn't exist yet, create it
            message_type = String # Default to string publisher
            if topic_name in self.message_type_map:
                message_type = self.message_type_map[topic_name] # otherwise 

            self.topic_publishers[topic_name] = self.create_publisher(message_type, topic_name, 10)
            self.get_logger().info(f"Created new publisher for topic: {topic_name}, type: {message_type}")

        return self.topic_publishers[topic_name]

    def handle_client(self, conn, addr):
        self.get_logger().info(f"Connected by {addr}")
        self.tcp_client = conn

        # Set a short timeout so we can periodically check if rclpy is still running.
        conn.settimeout(1.0)
        try:
            while rclpy.ok():
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    # No data arrived within 1 second, check if we're still running.
                    continue
                if not data:
                    # Client disconnected.
                    break

                # Process the received message.
                message = data.decode().strip()
                self.get_logger().info(f"Received from TCP: {message}")

                parts = message.split(';')
                if len(parts) < 2:
                    self.get_logger().error("Invalid message format. Expected 'topic_name;message_content'.")
                    continue

                topic_name = parts[0]
                content = ';'.join(parts[1:])
                publisher = self.get_or_create_publisher(topic_name)
                if not publisher:
                    continue
                
                if topic_name == "taranisenable":
                    self.enable_taranis()
                    continue
                    

                # Create and publish the message based on its type.
                message_type = self.message_type_map.get(topic_name, String)
                if message_type == String:
                    ros_msg = String()
                    ros_msg.data = content
                elif message_type == Float32MultiArray:
                    ros_msg = Float32MultiArray()
                    ang1 = float(parts[1])
                    ang2 = float(parts[2])
                    ang3 = float(parts[3])
                    ang4 = float(parts[4])
                    ang5 = float(parts[5])
                    ang6 = float(parts[6])
                    ros_msg.data = [ang1, ang2, ang3, ang4, ang5, ang6]
                else:
                    self.get_logger().error(f"Unsupported message type for topic: {topic_name}")
                    continue

                publisher.publish(ros_msg)
                self.get_logger().info(f"Published to ROS topic: {topic_name}")

        except Exception as e:
            self.get_logger().error(f"Error handling client: {e}")
        finally:
            self.get_logger().info(f"Closing connection with {addr}")
            conn.close()
            self.tcp_client = None

    def start_tcp_server(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable port reuse
            server_socket.bind((host, port))
            server_socket.listen()
            self.get_logger().info(f"Server listening on {host}:{port}...")

            try:
                while rclpy.ok():
                    # Use a timeout to allow periodic checking of rclpy.ok()
                    server_socket.settimeout(1.0)
                    try:
                        conn, addr = server_socket.accept()  # Accept a connection
                        if not rclpy.ok():
                            break
                        self.handle_client(conn, addr)  # Handle the client in the same thread
                    except socket.timeout:
                        continue  # Timeout reached; check rclpy.ok() again
            except Exception as e:
                if rclpy.ok():  # Avoid logging if shutdown has been called
                    self.get_logger().error(f"Error in TCP server: {e}")
            finally:
                if rclpy.ok():
                    self.get_logger().info("Shutting down TCP server.")

    def enable_taranis(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        custom_msg = DriveCommandMessage()
        custom_msg.controller_present = False
        custom_msg.ignore_drive_control = False
        custom_msg.drive_twist = twist_msg
        self.drive_publisher.publish(custom_msg)

shutdown_called = False

def main(args=None):
    global shutdown_called
    rclpy.init(args=args)
    node = TCPServer()

    # Run the TCP server in a separate thread
    tcp_thread = Thread(target=node.start_tcp_server, args=('127.0.0.1', 65432))
    tcp_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not shutdown_called:
            shutdown_called = True
            node.get_logger().info("Shutting down server...")
            node.destroy_node()
            #rclpy.shutdown()

        if tcp_thread.is_alive():
            tcp_thread.join()

if __name__ == '__main__':
    main()

