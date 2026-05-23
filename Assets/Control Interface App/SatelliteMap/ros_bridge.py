#!/usr/bin/env python3
import socket
import json
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from nav_autonomy_interface.action import Mission
from nav_autonomy_interface.msg import GPSWaypoint

class ROSBridge(Node):
    def __init__(self, rover_action_server='mission'):
        super().__init__('ros_bridge')
        self.action_client = ActionClient(self, Mission, rover_action_server)

        self.tcp_port = 5005
        self.tcp_host = '0.0.0.0'
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_sock.bind((self.tcp_host, self.tcp_port))
        self.tcp_sock.listen(1)        

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.unity_ip = '127.0.0.1'
        self.udp_port = 5006

        self.tcp_thread = threading.Thread(target=self._tcp_listen, daemon=True)
        self.tcp_thread.start()

        self.latest_telemetry = {
            "timestamp": 0.0,
            "mission_state": 0,
            "latitude": 0.0,
            "longitude": 0.0,
            "heading": 0.0
        }

        self.gps_sub = self.create_subscription(NavSatFix, 'gps/fix', self._gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self._odom_callback, 10)
        self.telemetry_timer = self.create_timer(0.2, self._broadcast_telemetry)


    def _gps_callback(self, msg):
        self.latest_telemetry["latitude"] = msg.latitude
        self.latest_telemetry["longitude"] = msg.longitude


    def _odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_telemetry["heading"] = math.degrees(yaw)


    def _broadcast_telemetry(self):
        self.latest_telemetry["timestamp"] = self.get_clock().now().nanoseconds / 1e9 # Convert to seconds
        try:
            payload = json.dumps(self.latest_telemetry).encode('utf-8')
            self.udp_sock.sendto(payload, (self.unity_ip, self.udp_port))
        except Exception as e:
            self.get_logger().error(f"Failed to transmit UDP telemetry: {e}")


    def _tcp_listen(self):
        self.get_logger().info(f"ROSBridge listening on {self.tcp_host}:{self.tcp_port}")
        while rclpy.ok():
            conn, addr = self.tcp_sock.accept()
            self.unity_ip = addr[0]
            with conn:
                self.get_logger().info(f"Connection from {addr}")
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                try:
                    mission = json.loads(data.decode('utf-8'))
                    self.get_logger().info(f"Received mission: {mission}")
                    self.send_mission(mission)
                except Exception as e:
                    self.get_logger().error(f"Failed to parse mission: {e}")


    def send_mission(self, mission):
        waypoints = mission.get('nav_waypoints', [])
        if not waypoints or len(waypoints) == 0:
            self.get_logger().warn("Received mission with no waypoints. Not sending to rover.")
            return

        goal_msg = Mission.Goal()
        goal_msg.search_object = mission.get('search_object', 0)
        goal_msg.search_pattern = mission.get('search_pattern', 0)
        goal_msg.search_param_1 = float(mission.get('search_param_1', 0.0))
        goal_msg.search_param_2 = float(mission.get('search_param_2', 0.0))
        goal_msg.nav_waypoints = []

        for wp in waypoints:
            waypoint = GPSWaypoint()
            waypoint.latitude = wp.get('latitude', 0.0)
            waypoint.longitude = wp.get('longitude', 0.0)
            goal_msg.nav_waypoints.append(waypoint)

        self.get_logger().info(f"Sending mission goal to rover...")
        self.get_logger().info(f"Goal msg: {goal_msg}")

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)


    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Mission goal rejected by rover.")
            return
        
        self.get_logger().info("Mission goal accepted by rover.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)


    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Mission completed with result: {result.ack}")

        if result.ack == Mission.Result.SUCCESS:
            self.latest_telemetry["mission_state"] = 6 # SUCCESS
            self.get_logger().info("Mission completed successfully!")
        elif result.ack == Mission.Result.FAILED:
            self.latest_telemetry["mission_state"] = 7 # FAILED
        else:
            self.latest_telemetry["mission_state"] = 9 # STOPPED


    def _feedback_callback(self, msg):
        self.latest_telemetry["mission_state"] = msg.feedback.mission_state
        

def main():
    rclpy.init()
    node = ROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
