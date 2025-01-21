import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Use appropriate message type
from geometry_msgs.msg import Twist  # Import Twist message type
from rover2_control_interface.msg import DriveCommandMessage
import time


class auton_controller(Node):
    speed = 0.1

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

        # Publisher for Twist messages to 'x'
        self.drive_publisher = self.create_publisher(DriveCommandMessage, 'command_control/ground_station_drive', 10)

    def start_control_loop(self):
        """Start a timer to publish the message at a specific interval for a duration."""
        interval = 0.1
        duration = 1.0
        start_time = time.time()
        
        
        def publish_message():
            elapsed_time = time.time() - start_time
            if elapsed_time < duration:
                if elapsed_time >= 0.9:
                    self.speed = 0.0

                twist_msg = Twist()
                twist_msg.linear.x = self.speed
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0

                custom_msg = DriveCommandMessage()
                custom_msg.controller_present = True
                custom_msg.ignore_drive_control = False
                custom_msg.drive_twist = twist_msg  # Embed the Twist message

                self.drive_publisher.publish(custom_msg)
                self.get_logger().info(f'Published to topic command_control/ground_station_drive')
            else:
                self.timer.cancel()  # Stop the timer

        self.timer = self.create_timer(interval, publish_message)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}" on auton_control')

        # Publish response message
        reply_msg = String()
        reply_msg.data = 'STARTING'
        self.response_publisher.publish(reply_msg)
        self.get_logger().info(f'Published: "{reply_msg.data}" to auton_control_response')

        self.start_control_loop()

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
