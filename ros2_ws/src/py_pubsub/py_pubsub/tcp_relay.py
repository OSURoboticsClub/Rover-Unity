import socket
import rclpy
from rclpy.node import Node
from custom_msg.msg import Sphere  # Replace with your actual custom message import
from std_msgs.msg import String  # Import for String message type
from threading import Thread


class TCPServer(Node):
    def __init__(self):
        super().__init__('tcp_server_with_ros2')
        self.publisher_ = self.create_publisher(Sphere, 'sphere_topic', 10)  # ROS 2 Publisher
        self.tcp_client = None  # Placeholder for TCP client connection
        self.get_logger().info('ROS 2 Publisher initialized.')

        # ROS 2 Subscriber for "gps_response"
        self.subscription = self.create_subscription(
            String,
            'auton_control_response',
            self.auton_control_response_callback,
            10
        )
        self.get_logger().info('Subscribed to "auton_control_response".')

    def auton_control_response_callback(self, msg):
        # Callback triggered when a message is received on "gps_response"
        if self.tcp_client:
            try:
                # Send the received message over the TCP socket
                self.tcp_client.sendall(msg.data.encode('utf-8'))
                self.get_logger().info(f"Sent message over TCP: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send message over TCP: {e}")
        else:
            self.get_logger().warn("No active TCP client. Message not sent.")

    def handle_client(self, conn, addr):
        self.get_logger().info(f"Connected by {addr}")
        self.tcp_client = conn  # Store the active TCP connection

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                # Log the received message
                message = data.decode()
                self.get_logger().info(f"Received: {message}")
                parts = message.split(";")
                if len(parts) < 3:
                    self.get_logger().error("Invalid message format. Expected 'cmd;latitude;longitude'.")
                    continue

                try:
                    lat = float(parts[1])
                    long = float(parts[2])
                    # Publish a Sphere message to ROS 2
                    sphere_msg = Sphere()
                    sphere_msg.cmd = parts[0]
                    sphere_msg.latitude = lat
                    sphere_msg.longitude = long
                    self.publisher_.publish(sphere_msg)
                    self.get_logger().info(f"Published Sphere: cmd={sphere_msg.cmd}, latitude={sphere_msg.latitude}, longitude={sphere_msg.longitude}")

                    # Send acknowledgment to the client
                    conn.sendall(b'Message received and Sphere published.')
                except ValueError:
                    self.get_logger().error("Invalid latitude or longitude format. Skipping message.")
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

