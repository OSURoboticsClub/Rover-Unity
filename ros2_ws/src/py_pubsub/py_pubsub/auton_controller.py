import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Use appropriate message type
from geometry_msgs.msg import Twist  # Import Twist message type
from rover2_control_interface.msg import DriveCommandMessage
import time
from geographiclib.geodesic import Geodesic


class auton_controller(Node):
    linear_speed = 0.1
    angular_speed = 0.0
    current_lat = 0.0
    current_lon = 0.0
    stop = False
    turning = False
    driving = False
    target_heading = 0.0

    def __init__(self):
        super().__init__('auton_controller')

        # Subscription to 'auton_control'
        self.subscription = self.create_subscription(
            String,
            'auton_control',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher for 'auton_control_response'
        self.response_publisher = self.create_publisher(String, 'auton_control_response', 10)

        # Publisher for Twist messages to 'command_control/ground_station_drive'
        self.drive_publisher = self.create_publisher(DriveCommandMessage, 'command_control/ground_station_drive', 10)

        # Unified control loop
        self.create_timer(0.1, self.control_loop)

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        geod = Geodesic.WGS84
        result = geod.Inverse(lat1, lon1, lat2, lon2)
        return result['azi1']

    def control_loop(self):
        """
        Unified control loop that handles both turning and driving.
        """
        if self.stop:
            self.get_logger().info("Stop flag detected. Stopping all actions.")
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.publish_drive_message()
            return

        if self.turning:
            self.get_logger().info("Turning to target heading...")
            # Placeholder for heading comparison logic
            current_heading = 0.0  # Replace with actual sensor data
            if abs(current_heading - self.target_heading) < 1.0:  # Example threshold
                self.get_logger().info("Target heading reached.")
                self.turning = False
                self.driving = True
            else:
                self.angular_speed = 0.2  # Example turning speed
                self.linear_speed = 0.0
                self.publish_drive_message()

        elif self.driving:
            self.get_logger().info("Driving to target location...")
            # Placeholder for driving logic
            # Add logic to calculate distance to target and adjust speeds
            self.linear_speed = 0.1  # Example driving speed
            self.angular_speed = 0.0
            self.publish_drive_message()

    def publish_drive_message(self):
        """
        Publishes a drive command with the current speed.
        """
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_speed

        custom_msg = DriveCommandMessage()
        custom_msg.controller_present = True
        custom_msg.ignore_drive_control = False
        custom_msg.drive_twist = twist_msg

        self.drive_publisher.publish(custom_msg)
        self.get_logger().info("Published drive command")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}" on auton_control')

        # Parse message
        try:
            parts = msg.data.split(';')
            command = parts[0]
            lat = float(parts[1])
            lon = float(parts[2])

            if command == "GOTO":
                self.get_logger().info(f"Command GOTO received with target lat: {lat}, lon: {lon}")
                self.stop = False
                self.turning = True
                self.driving = False
                self.target_heading = self.calculate_bearing(self.current_lat, self.current_lon, lat, lon)
            elif command == "STOP":
                self.get_logger().info("STOP command received. Stopping autonomous navigation.")
                self.stop = True
                self.turning = False
                self.driving = False
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}. Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = auton_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
