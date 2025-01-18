import socket
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import String
from rover2_control_interface.msg import GPSStatusMessage


class TCPServer(Node):
    def __init__(self):
        super().__init__('tcp_server_with_ros2')
        self.publisher_ = self.create_publisher(String, 'tcp_to_ros', 10)  # ROS 2 Publisher
        self.subscribers = []  # To store subscriber objects
        self.tcp_client = None  # Placeholder for TCP client connection
        self.get_logger().info('ROS 2 Publisher initialized.')

        # Add subscriptions to multiple topics
        self.add_subscription('test1', String)
        self.add_subscription('test2', String)
        self.add_subscription('tower/status/gps', GPSStatusMessage)  # Add more topics as needed

    def add_subscription(self, topic_name, message_type):
        # Create and store a subscription for each topic
        subscription = self.create_subscription(
            message_type,
            topic_name,
            self.ros_to_tcp_callback,
            10
        )
        self.subscribers.append(subscription)
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def ros_to_tcp_callback(self, msg):
        # Callback for messages received on ROS 2 topics
        if not(self.tcp_client):
            self.get_logger().warn("No active TCP client. Message not sent.")
            return
            
        try:
            # Determine message type and construct the string accordingly
            if isinstance(msg, String):  # If the message is of type std_msgs/String
                message_str = msg.data
            elif isinstance(msg, GPSStatusMessage):  # Replace with your actual custom message type
                # Construct a string from the custom message fields
                message_str = f"field1: {msg.rover_latitude}, field2: {msg.rover_longitude}"
            else:
                # Handle unknown message types
                message_str = f"Unsupported message type: {type(msg).__name__}"

            # Send the constructed string over TCP
            self.tcp_client.sendall(message_str.encode('utf-8'))
            self.get_logger().info(f"Sent message over TCP: {message_str}")
        except Exception as e:
            self.get_logger().error(f"Failed to send message over TCP: {e}")

    def handle_client(self, conn, addr):
        self.get_logger().info(f"Connected by {addr}")
        self.tcp_client = conn  # Store the active TCP connection

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                # Process the received message
                message = data.decode().strip()
                self.get_logger().info(f"Received from TCP: {message}")

                # Publish received message to ROS 2
                ros_msg = String()
                ros_msg.data = message
                self.publisher_.publish(ros_msg)
                self.get_logger().info(f"Published to ROS topic: {ros_msg.data}")

                # Acknowledge the client
                conn.sendall(b'Message received and published.')
        except Exception as e:
            self.get_logger().error(f"Error handling client: {e}")
        finally:
            self.get_logger().info(f"Closing connection with {addr}")
            conn.close()
            self.tcp_client = None  # Clear the active connection

    def start_tcp_server(self, host, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable port reuse
            server_socket.bind((host, port))
            server_socket.listen()
            self.get_logger().info(f"Server listening on {host}:{port}...")

            while rclpy.ok():
                conn, addr = server_socket.accept()  # Accept a connection
                self.handle_client(conn, addr)  # Handle the client in the same thread


def main(args=None):
    rclpy.init(args=args)
    node = TCPServer()

    # Run the TCP server in a separate thread
    tcp_thread = Thread(target=node.start_tcp_server, args=('127.0.0.1', 65432))
    tcp_thread.start()

    try:
        rclpy.spin(node)  # Spin the node to handle ROS 2 tasks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down server...")
    finally:
        tcp_thread.join()  # Wait for the TCP thread to finish
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

