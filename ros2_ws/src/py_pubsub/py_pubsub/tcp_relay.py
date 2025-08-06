import socket
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Float32
from rover2_control_interface.msg import GPSStatusMessage
from rover2_control_interface.msg import DriveCommandMessage
from rover2_control_interface.msg import TowerPanTiltControlMessage
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
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from rover2_status_interface.msg import NodesTopics
from rover2_status_interface.msg import ODriveStatus



# This node handles communication between the Unity app and ROS2
# I learned that it's technically not a relay, it's a bridge, but I'm not changing the name now :)

# We don't use this node for sending images to Unity anymore! We use gstreamer
# UDP Configuration for Image Transmission

PACKET_SIZE = 4096
HEADER_SIZE = 16
PAYLOAD_SIZE = PACKET_SIZE - HEADER_SIZE
NO_MSG_THRESH = 60
# Create UDP Socket

class TCPServer(Node):

    def __init__(self):
        super().__init__('tcp_relay')
        self.topic_publishers = {}  # Dictionary to store topic_name -> publisher

        # when we receive a message from Unity, sometimes it has to be converted to a ROS2 msg
        # this map tells what topics must be converted
        self.message_type_map = {
            'set_joint_angles': Float32MultiArray,
            'joy2': Joy,
            'command_control/ground_station_drive': DriveCommandMessage,
            'chassis/pan_tilt/control': TowerPanTiltControlMessage,
            'tower/pan_tilt/control': TowerPanTiltControlMessage
        } 

        self.subscribers = []
        self.tcp_client = None
        self.get_logger().info('TCP server initialized.')
        self.bridge = CvBridge()
        self.no_msg_count = 0;
        
        # topics we subscribe to will automatically  have their data sent over to Unity
        self.add_subscription('autonomous/auton_control_response', String)
        self.add_subscription('tower/status/gps', GPSStatusMessage)
        self.add_subscription('imu/data/heading', Float32)
        self.add_subscription('autonomous/simple_position', String)
        self.add_subscription('/joint_states', JointState)
        self.add_subscription('/nodetopiclisten', NodesTopics)
        self.add_subscription('/odrive_telem', ODriveStatus)

      
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)


    # def ros_img_callback(self, msg):
    #     self.send_image_over_udp(msg)


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

    

    def ros_to_tcp_callback(self, topic_name, msg): # Callback for messages received on ROS 2 topics
        if not(self.tcp_client):
            if self.no_msg_count >= NO_MSG_THRESH:
                self.no_msg_count = 0
                self.get_logger().warn("No active TCP client. Message not sent.")
            self.no_msg_count += 1
            return
            
        try: # Determine message type and construct the string accordingly
            message = topic_name + ";"
            if topic_name == "/odrive_telem":
                
                message += f"{msg.bus}?"
                for can_id in msg.id:
                    message += f"{can_id};"
                message += "?"
                for temp in msg.t_motor:
                    message+=f"{temp:.2f};"
                message += "?"
                for disarm in msg.disarm_reason:
                    message+=f"{disarm};"
                


            elif topic_name == "tower/status/gps":
                message += f"{msg.rover_latitude};{msg.rover_longitude}"
            elif topic_name =="/nodetopiclisten":
                for i in range(len(msg.topic_name)):
                    message += f"{msg.topic_name[i]},{msg.topic_status[i][0]};"
                
                message += "?"
                
                for i in range(len(msg.node_name)):
                    message += f"{msg.node_name[i]},{msg.node_status[i][0]};"

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
            #self.get_logger().info(f"Sent message over TCP: {message}")
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
                #self.get_logger().info(f"Received from TCP: {message}")

                parts = message.split(';')
                if len(parts) < 2:
                    self.get_logger().error("Invalid message format. Expected 'topic_name;message_content'.")
                    continue

                topic_name = parts[0]
                content = ';'.join(parts[1:])

                publisher = self.get_or_create_publisher(topic_name)
                if not publisher:
                    continue

                if topic_name == "joy2":
                    # can't remember what this is for, oops!
                    ros_msg = Joy()
                    ros_msg.axes = [0.0]
                    ros_msg.buttons = [0,0,0,0,0,0,0,0,0,0,0]
                    publisher.publish(ros_msg)
                    self.get_logger().info(f"Published to joy2")

                    ros_msg.buttons = [0,0,0,0,0,0,0,0,1,0,0]
                    publisher.publish(ros_msg)
                    self.get_logger().info(f"Published to joy2")
                    continue
                if topic_name == "command_control/ground_station_drive":
                    ros_msg = DriveCommandMessage()
                    twist_component = Twist()
                    linear = Vector3()
                    angular = Vector3()

                    ros_msg.controller_present = True if parts[1] == "True" else False
                    ros_msg.ignore_drive_control = True if parts[1] == "True" else False

                    linear.x = float(parts[3])
                    linear.y = 0.0
                    linear.z = 0.0

                    angular.x = 0.0
                    angular.y = 0.0
                    angular.z = float(parts[4])
                    
                    
                    twist_component.linear = linear
                    twist_component.angular = angular

                    ros_msg.drive_twist = twist_component

                    publisher.publish(ros_msg)
                    self.get_logger().info(f"Published to {topic_name}: {ros_msg.controller_present} {ros_msg.ignore_drive_control} {ros_msg.drive_twist}")

                if topic_name == "chassis/pan_tilt/control" or topic_name == "tower/pan_tilt/control":
                    ros_msg = TowerPanTiltControlMessage()
                    ros_msg.should_center = True if parts[1] == "True" else False
                    ros_msg.relative_pan_adjustment = int(parts[2])
                    ros_msg.relative_tilt_adjustment = int(parts[3])
                    ros_msg.hitch_servo_positive = True if parts[1] == "True" else False
                    ros_msg.hitch_servo_negative = True if parts[1] == "True" else False

                    publisher.publish(ros_msg)
                    self.get_logger().info(f"Published to {topic_name}: {ros_msg.should_center} {ros_msg.relative_pan_adjustment} {ros_msg.relative_tilt_adjustment}")

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
                #else:
                #    self.get_logger().error(f"Unsupported message type for topic: {topic_name}")
                #    continue

                publisher.publish(ros_msg)
                #self.get_logger().info(f"Published to {topic_name}: {ros_msg.data}")

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

shutdown_called = False

def main(args=None):
    global shutdown_called
    rclpy.init(args=args)
    node = TCPServer()

    # Run the TCP server in a separate thread
    tcp_thread_status = Thread(target=node.start_tcp_server, args=('127.0.0.1', 65432))
    tcp_thread_control = Thread(target=node.start_tcp_server, args=('127.0.0.1', 65433))

    tcp_thread_status.start()
    #tcp_thread_control.start()


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
