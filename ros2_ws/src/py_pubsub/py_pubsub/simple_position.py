import rclpy
from rclpy.node import Node
from rover2_control_interface.msg import DriveCommandMessage
from rover2_control_interface.msg import GPSStatusMessage
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import String

class SimplePosition(Node):
    current_heading = None
    current_speed = None
    current_latitude = None
    current_longitude = None
    latest_gps_latitude = None
    latest_gps_longitude = None
    pwm_to_meter_per_sec_multiplier = 2.0 # 2 m/s at 100% power

    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node has started')
        self.gps_subscription = self.create_subscription(GPSStatusMessage, 'tower/status/gps', self.gps_listener_callback, 10)
        self.imu_subscription = self.create_subscription(Float32, 'imu/data/heading', self.imu_heading_listener_callback, 10)
        self.drive_subscription = self.create_subscription(DriveCommandMessage, 'command_control/ground_station_drive', self.drive_listener_callback, 10)
        self.manual_position_subscription = self.create_subscription(String, 'autonomous/manually_set_position', self.manual_set_position_listener_callback, 10)

        self.publisher = self.create_publisher(String, 'autonomous/simple_position', 10)

        self.control_timer = self.create_timer(0.1, self.publish_loop)

    def publish_loop(self):
        # calculate new position
        if self.current_latitude == None:
            return
        distance_covered = 0.1 * self.current_speed
        geod = Geodesic.WGS84

        new_pos = geod.Direct(self.current_latitude, self.current_longitude, self.current_heading, distance_covered)
        self.current_latitude = new_pos['lat2']
        self.current_longitude = new_pos['lon2']

        self.get_logger().info(f'Distance covered: {distance_covered}. New lat: {self.current_latitude}, new lon: {self.current_longitude}')
        msg = str(self.current_latitude) + ";" + str(self.current_longitude)
        self.publisher.publish(msg)

    def gps_listener_callback(self, msg):
        self.latest_gps_latitude = msg.rover_latitude
        self.latest_gps_longitude = msg.rover_longitude

    def imu_heading_listener_callback(self, msg):
        self.current_heading = msg.data

    def drive_listener_callback(self, msg):
        self.current_speed = msg.drive_twist.linear.x * self.pwm_to_meter_per_sec_multiplier
        
    def manual_set_position_listener_callback(self, msg):
        parts = msg.data.split(';')
        lat = float(parts[0])
        lon = float(parts[1])
        self.get_logger().info(f'Received: "{msg.data}" on manually_set_position')
        self.current_latitude = lat
        self.current_longitude = lon

def main(args=None):
    rclpy.init(args=args)
    node = SimplePosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
