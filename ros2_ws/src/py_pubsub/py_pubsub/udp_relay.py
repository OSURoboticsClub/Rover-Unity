import socket
import rclpy
from rclpy.node import Node
from threading import Thread
from rover2_control_interface.msg import DriveCommandMessage
from rover2_control_interface.msg import TowerPanTiltControlMessage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# UDP Control Relay Node
# Handles low-latency control commands (drive, pan/tilt) via UDP
# Works in tandem with the TCP relay for status updates

class UDPControlRelay(Node):
    
    def __init__(self):
        super().__init__('udp_control_relay')
        self.topic_publishers = {}  # Dictionary to store topic_name -> publisher
        
        # Control message type mapping
        self.control_message_types = {
            'command_control/ground_station_drive': DriveCommandMessage,
            'chassis/pan_tilt/control': TowerPanTiltControlMessage,
            'tower/pan_tilt/control': TowerPanTiltControlMessage,
            'joy2': Joy,
            'set_joint_angles': Float32MultiArray
        }
        
        self.udp_socket = None
        self.client_address = None
        self.get_logger().info('UDP Control Relay initialized.')
        
    def get_or_create_publisher(self, topic_name):
        """Create publisher for control topics if it doesn't exist"""
        if topic_name not in self.topic_publishers:
            if topic_name not in self.control_message_types:
                self.get_logger().error(f"Unknown control topic: {topic_name}")
                return None
                
            message_type = self.control_message_types[topic_name]
            self.topic_publishers[topic_name] = self.create_publisher(message_type, topic_name, 10)
            self.get_logger().info(f"Created UDP publisher for: {topic_name}, type: {message_type}")
            
        return self.topic_publishers[topic_name]
    
    def process_control_message(self, message):
        """Process incoming UDP control messages"""
        try:
            parts = message.split(';')
            if len(parts) < 2:
                self.get_logger().error("Invalid UDP message format. Expected 'topic_name;message_content'.")
                return
                
            topic_name = parts[0]
            
            # Only handle control topics
            if topic_name not in self.control_message_types:
                self.get_logger().warn(f"Non-control topic received on UDP: {topic_name}")
                return
                
            publisher = self.get_or_create_publisher(topic_name)
            if not publisher:
                return
                
            # Handle different control message types
            if topic_name == "joy2":
                self.handle_joy_message(publisher)
                
            elif topic_name == "command_control/ground_station_drive":
                self.handle_drive_command(publisher, parts)
                
            elif topic_name in ["chassis/pan_tilt/control", "tower/pan_tilt/control"]:
                self.handle_pan_tilt_command(publisher, parts)
                
            elif topic_name == "set_joint_angles":
                self.handle_joint_angles(publisher, parts)
                
        except Exception as e:
            self.get_logger().error(f"Error processing UDP control message: {e}")
    
    def handle_joy_message(self, publisher):
        """Handle joy2 messages"""
        # First message
        ros_msg = Joy()
        ros_msg.axes = [0.0]
        ros_msg.buttons = [0,0,0,0,0,0,0,0,0,0,0]
        publisher.publish(ros_msg)
        
        # Second message
        ros_msg.buttons = [0,0,0,0,0,0,0,0,1,0,0]
        publisher.publish(ros_msg)
        self.get_logger().debug("Published joy2 messages")
    
    def handle_drive_command(self, publisher, parts):
        """Handle drive command messages"""
        if len(parts) < 5:
            self.get_logger().error("Invalid drive command format")
            return
            
        ros_msg = DriveCommandMessage()
        twist_component = Twist()
        linear = Vector3()
        angular = Vector3()
        
        ros_msg.controller_present = parts[1].lower() == "true"
        ros_msg.ignore_drive_control = parts[2].lower() == "true"
        
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
        self.get_logger().debug(f"Published drive command: controller={ros_msg.controller_present}, "
                              f"ignore={ros_msg.ignore_drive_control}, "
                              f"linear_x={linear.x}, angular_z={angular.z}")
    
    def handle_pan_tilt_command(self, publisher, parts):
        """Handle pan/tilt control messages"""
        if len(parts) < 6:
            self.get_logger().error("Invalid pan/tilt command format")
            return
            
        ros_msg = TowerPanTiltControlMessage()
        ros_msg.should_center = parts[1].lower() == "true"
        ros_msg.relative_pan_adjustment = int(parts[2])
        ros_msg.relative_tilt_adjustment = int(parts[3])
        ros_msg.hitch_servo_positive = parts[4].lower() == "true"
        ros_msg.hitch_servo_negative = parts[5].lower() == "true"
        
        publisher.publish(ros_msg)
        self.get_logger().debug(f"Published pan/tilt: center={ros_msg.should_center}, "
                              f"pan={ros_msg.relative_pan_adjustment}, "
                              f"tilt={ros_msg.relative_tilt_adjustment}")
    
    def handle_joint_angles(self, publisher, parts):
        """Handle joint angle messages"""
        if len(parts) < 7:  # topic + 6 angles
            self.get_logger().error("Invalid joint angles format")
            return
            
        ros_msg = Float32MultiArray()
        try:
            angles = [float(parts[i]) for i in range(1, 7)]
            ros_msg.data = angles
            publisher.publish(ros_msg)
            self.get_logger().debug(f"Published joint angles: {angles}")
        except ValueError as e:
            self.get_logger().error(f"Error parsing joint angles: {e}")
    
    def start_udp_server(self, host, port):
        """Start UDP server for control messages"""
        self.get_logger().info(f"Starting UDP server thread...")
        try:
            self.get_logger().info(f"Creating UDP socket...")
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            self.get_logger().info(f"Binding to {host}:{port}...")
            self.udp_socket.bind((host, port))
            self.udp_socket.settimeout(1.0)  # 1 second timeout for periodic checks
            
            self.get_logger().info(f"UDP Control Server successfully bound to {host}:{port}")
            self.get_logger().info(f"Socket details: {self.udp_socket.getsockname()}")
            self.get_logger().info(f"Starting main UDP receive loop...")
            
            packet_count = 0
            while rclpy.ok():
                try:
                    data, addr = self.udp_socket.recvfrom(1024)
                    packet_count += 1
                    
                    self.get_logger().info(f"[Packet {packet_count}] Received {len(data)} bytes from {addr}")
                    
                    # Store client address for potential responses
                    if self.client_address != addr:
                        self.client_address = addr
                        self.get_logger().info(f"New UDP client connected: {addr}")
                    
                    message = data.decode('utf-8').strip()
                    self.get_logger().info(f"Message content: '{message}'")
                    
                    # Process the control message
                    self.process_control_message(message)
                    
                except socket.timeout:
                    # Timeout occurred, check if we should continue
                    continue
                except Exception as e:
                    if rclpy.ok():
                        self.get_logger().error(f"UDP server receive error: {e}")
                        import traceback
                        self.get_logger().error(f"Traceback: {traceback.format_exc()}")
                        
        except Exception as e:
            self.get_logger().error(f"Failed to start UDP server: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
        finally:
            if self.udp_socket:
                self.get_logger().info("Closing UDP socket...")
                self.udp_socket.close()
                self.get_logger().info("UDP Control Server shut down")

def main(args=None):
    rclpy.init(args=args)
    node = UDPControlRelay()
    
    # Start UDP server in a separate thread
    udp_thread = Thread(target=node.start_udp_server, args=('127.0.0.1', 65434))
    udp_thread.daemon = True  # Daemon thread will exit when main program exits
    udp_thread.start()
    
    # Give the UDP server a moment to start
    import time
    time.sleep(0.5)
    
    # Check if thread is alive
    if udp_thread.is_alive():
        node.get_logger().info("UDP thread started successfully")
    else:
        node.get_logger().error("UDP thread failed to start!")
    
    try:
        node.get_logger().info("Starting ROS2 spin...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received KeyboardInterrupt")
    except Exception as e:
        node.get_logger().error(f"Error in main: {e}")
    finally:
        node.get_logger().info("Shutting down UDP Control Relay...")
        node.destroy_node()
        rclpy.shutdown()
        
        # Wait for UDP thread to finish
        if udp_thread.is_alive():
            node.get_logger().info("Waiting for UDP thread to finish...")
            udp_thread.join(timeout=2.0)

if __name__ == '__main__':
    main()