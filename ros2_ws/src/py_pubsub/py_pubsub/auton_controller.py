import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage
from rover2_control_interface.msg import GPSStatusMessage
from rover2_control import aruco_scan
from rover2_control import geographic_functions
#from rover2_control import bottle_detect
from rover2_status_interface.msg import LED
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
#from geographiclib.geodesic import Geodesic
from transforms3d.euler import quat2euler
from dataclasses import dataclass
#from rclpy.qos import QoSProfile, QoSReliabilityPolicy
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
import json
from dataclasses import asdict
import cv2
#import bottle_detect

@dataclass
class Location:
    latitude: float
    longitude: float

class auton_controller(Node):
    state = "stopped"
    led_state = "green"
    
    # waypoint_destination = None
    target_coordinate = None
    # curr_point_destination = None
    latest_img_frame = None
    rover_position = Location(44.56726, -123.27363)
    current_heading = 0.0
    target_heading = None
    offset = None
    time_driving = 0.0
    time_looking_for_item = None
    item_searching_for = None
    target_turning_velocity = 0.0
    curr_turning_velocity = 0.0
    pause_time = None
    bottle_detector = None
    camera = None

    control_timer = None 
    vel_control_loop_timer = None
    led_timer = None
    camera_timer = None


    def __init__(self):
        super().__init__('auton_controller')
        #self.bottle_detector = bottle_detect.bottle_detector()
        self.control_subscription = self.create_subscription( String, 'autonomous/auton_control', self.control_listener_callback, 10)
        self.gps_subscription = self.create_subscription(GPSStatusMessage, 'tower/status/gps', self.gps_listener_callback, 10)
        #self.imu_subscription = self.create_subscription(Imu, 'imu/data', self.imu_listener_callback, 10)
        self.imu_subscription = self.create_subscription(Float32, 'imu/data/heading', self.imu_heading_listener_callback, 10)
        #qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        #self.img_subscription = self.create_subscription(Image, '/cameras/main_navigation/image_256x144', self.ros_img_callback, qos_profile)

        self.response_publisher = self.create_publisher(String, 'autonomous/auton_control_response', 10)
        self.drive_publisher = self.create_publisher(DriveCommandMessage, 'command_control/ground_station_drive', 10)
        self.led_publisher = self.create_publisher(LED, 'autonomous_LED/color', 10)
        #self.publish_led_message(,0,0)

        self.camera_timer = self.create_timer(0.05, self.camera_loop)
        self.get_logger().info("Auton controller initialized")
        
    def control_loop(self):
        if self.state == "stopped":
            self.publish_drive_message(0.0, 0.0)
            self.control_timer.cancel()
            self.control_timer = None
            if self.vel_control_loop_timer is not None:
                self.vel_control_loop_timer.cancel()
                self.vel_control_loop_timer = None
            self.target_coordinate = None
            self.target_heading = None
            self.time_looking_for_item = None
            self.pause_time = None
            self.get_logger().info("Stopped")
            self.publish_log_msg("Stopped autonomous control")
            return

        if self.state == "turning":

            heading_error = self.get_heading_error()
            self.get_logger().info("Turning. Target: " + f"{self.target_heading:.1f}" + ". Current: " + f"{self.current_heading:.1f}" + ", Error: " + f"{heading_error:.1f}")
            if abs(heading_error) < 2.5:  # Example threshold
                self.get_logger().info("Target heading reached.")
                self.publish_log_msg("Reached target heading. Now driving")
                self.state = "driving"
            else:
                angular_speed = 0.4 # rad/s
                if heading_error > 0:
                    angular_speed *= -1
                #if abs(heading_error) < 30: # slow down on approach
                    #angular_speed *= 0.6
                self.publish_drive_message(0.0, angular_speed) 

        elif self.state == "driving":
            self.target_heading = geographic_functions.get_target_heading(self.rover_position, self.curr_point_destination)
            #distance_to_nearest_point = geographic_functions.get_distance_to_location(self.rover_position, self.curr_point_destination)
            distance_to_waypoint = geographic_functions.get_distance_to_location(self.rover_position, self.target_coordinate)
            curvature = geographic_functions.compute_curvature(self.curr_point_destination, self.get_heading_error())


            if distance_to_waypoint < 9.0:
                self.set_next_dest()
                if self.curr_point_destination is None:
                    # send message to unity
                    # maybe get back the FIND command
                    self.state = "stopped"
                    self.get_logger().info("Reached destination. Stopping...")
                    self.publish_log_msg("stopped")
                    return
            elif distance_to_waypoint < 13.0:
                self.get_logger().info("getting_close")
                self.publish_log_msg("getting_close")

            linear = 0.65
            angular = -curvature * linear * 1.7
            if angular > 0.6:
                angular = 0.6
            elif angular < -0.6:
                angular = -0.6
            
            log1 = "Driving. Dist to target: " + f"{distance_to_waypoint:.0f}. Curv: " + f"{curvature:0.1f}" + ". Angular: " + str(angular) + ". "
            heading_log = "Target H: " + f"{self.target_heading:.1f}, " + "Current H: " + f"{self.current_heading:.1f}"
            self.get_logger().info(log1 + heading_log)
            self.publish_drive_message(linear, angular)
        
        elif self.state == "scanning":
            # turn until an aruco tag is found
            if self.vel_control_loop_timer is None:
                self.vel_control_loop_timer = self.create_timer(0.1, self.vel_control_loop)

            if self.latest_img_frame is None:
                self.get_logger().info("No images from camera feed yet")
                return

            if self.time_looking_for_item is None:
                self.time_looking_for_item = 0.0
            if self.time_looking_for_item >= 11.0:
                self.get_logger().info(f"Timed out looking for a(n) {self.item_searching_for}")
                self.state = "stopped"
                return
            
            item_location_in_img = None
            width = None
            if self.item_searching_for == "ARUCO":
                item_location_in_img, width = aruco_scan.detect_first_aruco_marker(self, self.latest_img_frame)
            # elif self.item_searching_for == "bottle":
            #     item_location_in_img, width = self.bottle_detector.get_bottle(self.latest_img_frame)

            if item_location_in_img == None:
                self.time_looking_for_item += 0.1
                self.get_logger().info(f"Looking for {self.item_searching_for} for {self.time_looking_for_item:.2f} seconds")
                self.target_turning_velocity = 0.3
                return
            
            self.time_looking_for_item = 0.0
            angular_vel = 0.3
            if abs(item_location_in_img - 0.5) < 0.2:
                angular_vel = 0.25 # slow down on approach

            if item_location_in_img > 0.4 and item_location_in_img < 0.6:
                #self.get_logger().info(f"Pointed towards ARUCO, should now drive forward")
                self.curr_turning_velocity = 0.0
                angular_vel = 0.0
                if self.pause_time is not None:
                    if self.pause_time >= 1.0:
                        self.state = "driving to item"
                        self.get_logger().info(f"Now driving to {self.item_searching_for}")
                        self.vel_control_loop_timer.cancel()
                        self.vel_control_loop_timer = None
                        self.time_looking_for_item = None
                    else:
                        self.pause_time += 0.1
                else:
                    self.pause_time = 0.0
            elif item_location_in_img <= 0.4:
                #self.get_logger().info(f"ARUCO is to the left, turning left")
                angular_vel *= 1 # not necessary but helps understand the code
            else:
                #self.get_logger().info(f"ARUCO is to the right, turning right")
                angular_vel *= -1
            
            msg = f"{self.item_searching_for} is at: {item_location_in_img:.2f}. Width: {width:0.2f}. Target Angular: {angular_vel:.2f}"
            self.get_logger().info(msg + f" Curr angular: {self.curr_turning_velocity:.2f}")
            #self.target_turning_velocity = angular_vel

        elif self.state == "driving to item":
            linear_vel = 0.3
            angular = 0.0
            item_location_in_img, width = None, None
            if self.item_searching_for == "ARUCO":
                item_location_in_img, width = aruco_scan.detect_first_aruco_marker(self, self.latest_img_frame)
            # elif self.item_searching_for == "bottle":
            #     item_location_in_img, width = self.bottle_detector.get_bottle(self.latest_img_frame)

            if item_location_in_img == None:
                self.get_logger().info(f"Lost the {self.item_searching_for}")
                self.state = "scanning"
                return
            
            width_threshold = 0.16
            if self.item_searching_for == "bottle":
                width_threshold = 0.12
            if width > width_threshold:
                self.get_logger().info(f"Arrived")
                self.led_timer = self.create_timer(0.6, self.blinking_led_loop)
                self.publish_led_message(0,255,0)
                self.state = "stopped"
                return

            if item_location_in_img > 0.45 and item_location_in_img < 0.55:
                #self.get_logger().inscp ~/Documents/GitHub/Rover-Unity/ros2_ws/src/py_pubsub/py_pubsub/auton_controller.py makemorerobot@192.168.1.101:~/Rover_2023_2024/software/ros_packages/rover2_control/rover2_control/auton_controller.pyfo(f"On track towards ARUCO")
                hi = 4
            elif item_location_in_img <= 0.25:
                angular = 0.35
            elif item_location_in_img <= 0.45:
                #self.get_logger().info(f"ARUCO is to the left, turning left")
                angular = 0.2
            elif item_location_in_img < 0.75:
                angular = -0.2
            else:
                #self.get_logger().info(f"ARUCO is to the right, turning right")
                angular = -0.35
            

            msg = f"Driving towards {self.item_searching_for}. Location: {item_location_in_img:.2f}. Width: {width:0.2f}. Angular: {angular:.2f}"
            self.get_logger().info(msg)
            # self.publish_drive_message(linear_vel, angular) 

    def vel_control_loop(self):
        if self.curr_turning_velocity < self.target_turning_velocity:
            self.curr_turning_velocity += 0.03
        elif self.curr_turning_velocity > self.target_turning_velocity:
            self.curr_turning_velocity -= 0.03

        if abs(self.curr_turning_velocity - self.target_turning_velocity) < 0.04:
            self.curr_turning_velocity = self.target_turning_velocity
        self.publish_drive_message(0.0, self.curr_turning_velocity) 

    def control_listener_callback(self, msg):
        """Listens to auton_control topic for commands"""
        self.get_logger().info(f'Received: "{msg.data}" on auton_control')

        try:
            parts = msg.data.split(';')
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}. Error: {e}")
            return

        command = parts[0]

        if command == "GOTO":
            lat = float(parts[1])
            lon = float(parts[2])
            self.get_logger().info(f"GOTO received with target lat: {lat}, lon: {lon}")
            self.target_coordinate = Location(lat, lon)
            self.publish_led_message(255, 0, 0)
            if self.state != "driving":
                self.state = "turning"
                if self.led_timer is not None:
                    self.led_timer.cancel()
                    self.led_timer = None
                
                if self.control_timer is None:
                    self.control_timer = self.create_timer(0.1, self.control_loop)
            
            self.target_heading = geographic_functions.get_target_heading(self.rover_position, self.target_coordinate)
            #self.subpoints = geographic_functions.get_points_along_line(self.rover_position, self.waypoint_destination, self.target_heading)
            #msg = "subpoints;" + json.dumps([asdict(loc) for loc in self.subpoints])
            #self.publish_log_msg(msg)
            #self.set_next_dest()
        elif command == "FIND":
            self.item_searching_for = parts[1]
            self.state = "scanning"
            self.get_logger().info(f"Finding " + self.item_searching_for)
            if self.control_timer is None:
                self.control_timer = self.create_timer(0.1, self.control_loop)
        elif command == "STOP":
            self.get_logger().info("STOP command received. Stopping autonomous navigation.")
            self.publish_led_message(0, 0, 255)
            if self.led_timer is not None:
                self.led_timer.cancel()
                self.led_timer = None
            self.state = "stopped"
        else:
            self.get_logger().warn(f"Unknown command: {command}")
    
    def publish_led_message(self, red, green, blue):
        #self.get_logger().info(f"Published LED msg. R:{red}, G:{green}, B:{blue}")
        led_msg = LED()
        led_msg.red = red
        led_msg.green = green
        led_msg.blue = blue
        self.led_publisher.publish(led_msg)
        if green == 255 and red == 0 and blue == 0:
            self.led_state = "green"
        elif green == 0 and red == 0 and blue == 0:
            self.led_state = "black"
        else:
            self.led_state = "other"

    def blinking_led_loop(self):
        if self.led_state == "green":
            self.publish_led_message(0, 0, 0)
        else:
            self.publish_led_message(0, 255, 0)

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
        self.drive_publisher.publish(custom_msg)

    def publish_log_msg(self, text):
        msg = String()
        msg.data = text
        self.response_publisher.publish(msg)

    def get_heading_error(self):
        """Returns the shortest signed heading error in degrees."""
        error = self.target_heading - self.current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error
    
    # def set_next_dest(self):
    #     if self.subpoints is not None and len(self.subpoints) > 0:
    #         self.curr_point_destination = self.subpoints[0]
    #         self.subpoints.pop(0)
    #     elif self.curr_point_destination != self.waypoint_destination:
    #         self.curr_point_destination = self.waypoint_destination
    #     else:
    #         self.curr_point_destination = None
    #     self.get_logger().info("Set new dest: " + str(self.curr_point_destination))
    #     if self.curr_point_destination is not None:
    #         self.publish_log_msg("nextdest;" + str(self.curr_point_destination.latitude) + ";" + str(self.curr_point_destination.longitude))
    
    def imu_heading_listener_callback(self, msg):
        """Listens to auton_control topic for commands"""
        self.current_heading = msg.data
        if self.current_heading > 360.0:
            self.current_heading -= 360.0
        elif self.current_heading < 0.0:
            self.current_heading += 360.0
        #self.get_logger().info(f"Received heading: " + str(self.current_heading))

    def gps_listener_callback(self, msg):
        """Receive GPS data"""
        self.rover_position.latitude = msg.rover_latitude
        self.rover_position.longitude = msg.rover_longitude

    # def ros_img_callback(self, msg):
    #     self.latest_img_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_loop(self):
        if self.camera == None:
            self.camera = cv2.VideoCapture('/dev/video10')

        if not self.camera.isOpened():
            self.get_logger().info("Camera not opened yet")
            return

        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().info("Failed to get camera image")
            return
        
        self.latest_img_frame = frame


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

# def imu_listener_callback(self, msg):
#     """Receive IMU data, convert to Euler Angles"""
#     quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

#     euler = quat2euler(quat, axes='sxyz')  # 'sxyz' is a common rotation convention
#     roll, pitch, yaw = euler
#     roll_degrees = roll * 57.2958

#     if self.offset == None:
#         self.current_heading = 18.0  # The rover should start pointing in alignment with Merryfield
#         self.get_logger().info(f'Heading should be 18')
#         self.get_logger().info(f'Current heading is: ' + str(roll_degrees))
#         self.get_logger().info(f'Thus the offset is: ' + str(18.0 - roll_degrees))
#         self.offset = 18.0 - roll_degrees # At the start, determine the offset in degrees of the rover if the IMU is off
#     else:
#         new_heading = roll_degrees + 180.0
#         if new_heading > 180.0:
#             new_heading -= 360.0
#         #self.get_logger().info(f'heading is: ' + str(new_heading))
#         self.current_heading = new_heading
#         # if(self.current_heading < -180.0):
#         #     self.current_heading += 360
#         # if(self.current_heading > 180.0):
#         #     self.current_heading -= 360