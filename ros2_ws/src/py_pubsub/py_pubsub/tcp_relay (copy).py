import socket
import rclpy
from rclpy.node import Node
from custom_msg.msg import Sphere  # Replace with your actual custom message import
from threading import Thread

class TCPServer(Node):
    def __init__(self):
        super().__init__('tcp_server_with_ros2')
        self.publisher_ = self.create_publisher(Sphere, 'sphere_topic', 10)  # ROS 2 Publisher
        self.get_logger().info('ROS 2 Publisher initialized.')

    def handle_client(self, conn, addr):
        self.get_logger().info(f"Connected by {addr}")
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                # Log the received message
                message = data.decode()
                self.get_logger().info(f"Received: {message}")
                parts = message.split(";")
                lat = float(parts[1])
                long = float(parts[2])
                # Publish a Sphere message to ROS 2
                sphere_msg = Sphere()
                sphere_msg.cmd = parts[0]  # Assuming the message contains a numeric center
                sphere_msg.latitude = lat
                sphere_msg.longitude = long           # Example radius
                self.publisher_.publish(sphere_msg)
                self.get_logger().info(f"Published Sphere: center={sphere_msg.cmd}, radius={sphere_msg.latitude}")

                # Send acknowledgment to the client
                conn.sendall(b'Message received and Sphere published.')
        except Exception as e:
            self.get_logger().error(f"Error handling client: {e}")
        finally:
            conn.close()

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

