import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage
#from rover2_control_interface.msg import GPSStatusMessage
from sensor_msgs.msg import Imu
import time
from geographiclib.geodesic import Geodesic
from transforms3d.euler import quat2euler
# pip install numpy-quaternion


class auton_controller(Node):
    current_lat = 0.0
    current_lon = 0.0
    target_lat = 0.0
    target_lon = 0.0
    current_heading = 0.0
    target_heading = None
    state = "stopped"
    control_timer = None 
    offset = None

    def __init__(self):
        super().__init__('auton_controller')

        self.control_subscription = self.create_subscription( String, 'auton_control', self.control_listener_callback, 10)
        #self.gps_subscription = self.create_subscription(GPSStatusMessage, 'tower/status/gps', self.gps_listener_callback, 10)
        self.gps_subscription = self.create_subscription(Imu, 'imu/data', self.imu_listener_callback, 10)

        self.response_publisher = self.create_publisher(String, 'auton_control_response', 10)
        self.drive_publisher = self.create_publisher(DriveCommandMessage, 'command_control/ground_station_drive', 10)


    def publish_drive_message(self, linear_speed, angular_speed):
        """Publish the current linear and angular speed to drivetrain"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angular_speed
        custom_msg = DriveCommandMessage()
        custom_msg.controller_present = True
        custom_msg.ignore_drive_control = False
        custom_msg.drive_twist = twist_msg
        #self.drive_publisher.publish(custom_msg)

    def get_target_heading(self):
        geod = Geodesic.WGS84
        result = geod.Inverse(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        return result['azi1']

    def publish_log_msg(self, text):
        msg = String()
        msg.data = text
        self.response_publisher.publish(msg)

    def get_heading_error(self):
        return self.current_heading - self.target_heading

    def control_loop(self):
        if self.state == "stopped":
            self.publish_drive_message(0.0, 0.0)
            self.control_timer.cancel()
            self.control_timer = None
            self.publish_log_msg("Stopped autonomous control")
            return

        if self.target_heading == None:
            self.target_heading = self.get_target_heading()
            self.get_logger().info("Set target heading as: " + str(self.target_heading))

        if self.state == "turning":
            heading_error = self.get_heading_error()
            self.get_logger().info("Turning to target heading (" + str(self.target_heading) + "). Current heading: " + str(self.current_heading))
            if abs(heading_error) < 5.0:  # Example threshold
                self.get_logger().info("Target heading reached.")
                self.publish_log_msg("Reached target heading. Now driving")
                self.state = "driving"
            else:
                angular_speed = 0.5 # rad/s
                if heading_error < 0:
                    angular_speed *= -1
                if abs(heading_error) < 30: # slow down on approach
                    angular_speed *= 0.4
                self.publish_drive_message(0.0, angular_speed) 

        elif self.state == "driving":
            self.get_logger().info("Driving to target location...")
            self.linear_speed = 0.1  # Example driving speed
            self.angular_speed = 0.0
            self.publish_drive_message()

    def control_listener_callback(self, msg):
        """Listens to auton_control topic for commands"""
        self.get_logger().info(f'Received: "{msg.data}" on auton_control')

        try:
            parts = msg.data.split(';')
            command = parts[0]
            lat = float(parts[1])
            lon = float(parts[2])

            if command == "GOTO":
                self.get_logger().info(f"Command GOTO received with target lat: {lat}, lon: {lon}")
                self.state = "turning"
                if self.control_timer is not None:
                    self.control_timer.cancel()
                self.control_timer = self.create_timer(0.1, self.control_loop)
            elif command == "STOP":
                self.get_logger().info("STOP command received. Stopping autonomous navigation.")
                self.state = "stopped"
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}. Error: {e}")

    def imu_listener_callback(self, msg):
        """Receive IMU data, convert to Euler Angles"""
        self.get_logger().info(f'Received data on imu/data')
        quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

        euler = quat2euler(quat, axes='sxyz')  # 'sxyz' is a common rotation convention
        roll, pitch, yaw = euler
        roll_degrees = roll * 57.2958

        if self.offset == None:
            self.current_heading = 0  # The rover should start pointing North
            self.offset = roll_degrees # At the start, determine the offset in degrees of the rover if the IMU is off
        else:
            self.current_heading = roll_degrees - self.offset

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
