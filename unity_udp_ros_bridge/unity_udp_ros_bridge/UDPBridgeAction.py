import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_action

from unity_udp_ros_bridge.UDPServer import UDPServer
from threading import Thread, Lock

import json
import socket
from array import array

class UDPBridgeAction(Node):
    """ROS2 Node for handling action calls via UDP"""

    client_cache: dict[str, ActionClient]

    def __init__(self):
        super().__init__('udp_action_bridge')

        self.action_type_cache = {}

        self.client_cache = {}

        # Track pending action calls: request_id -> [future, addr]
        self.pending_requests = {}
        self.pending_lock = Lock()

        self.udp_server = UDPServer('127.0.0.1', 65439, logger=self.get_logger())

        self.udp_thread = Thread(
            target=self.udp_server.start,
            args=(self.process_udp_message,)
        )
        self.udp_thread.daemon = True
        self.udp_thread.start()

        self.get_logger().info("UDPBridgeAction node initialized on port 65439")

    def set_fields_from_dict(self, msg_obj, data_dict):
        """Recursively set message fields from a nested dictionary"""
        for key, value in data_dict.items():
            if hasattr(msg_obj, key):
                attr = getattr(msg_obj, key)
                if hasattr(attr, '__slots__') and isinstance(value, dict):
                    self.set_fields_from_dict(attr, value)
                else:
                    field_type = type(attr)
                    setattr(msg_obj, key, field_type(value))
            else:
                self.get_logger().warning(
                    f"Message {type(msg_obj).__name__} has no field '{key}'"
                )

    def dict_from_message(self, msg_obj):
        """Convert a ROS message to a dictionary"""
        result = {}
        for field in msg_obj.__slots__:
            value = getattr(msg_obj, field)
            if hasattr(value, '__slots__'):
                result[field] = self.dict_from_message(value)
            elif isinstance(value, array):
                result[field] = [
                    self.dict_from_message(item) if hasattr(item, '__slots__') else item
                    for item in value
                ]
            else:
                result[field] = value
        return result

    def process_udp_message(self, data, addr):
        """
        Handle incoming UDP action call requests

        Expected JSON format:
        {
            "action": "move_forward",
            "actionType: "actions/Action",
            "request": {},
            "result": {},
            "feedback": {}
        }
        """
        try:
            message = data.decode('utf-8').strip()
            payload = json.loads(message)

            action_name = payload.get("action")
            action_type = payload.get("actionType")
            request_data = payload.get("request", {})

            if not action_name or not action_type:
                self.get_logger().error(
                    "JSON missing required keys 'action' or 'actionType'"
                )
                return

            if action_type not in self.action_type_cache:
                self.action_type_cache[action_type] = get_action(action_type)

            action_class = self.action_type_cache[action_type]

            client_key = f"{action_name}:{action_type}"
            if client_key not in self.client_cache:
                self.client_cache[client_key] = ActionClient(self, action_class, action_name)
                self.get_logger().info(
                    f"Created action client for {action_name} with type {action_type}"
                )

            client = self.client_cache[client_key]

            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Action server {action_name} not available")

                return

            goal = action_class.Goal()
            self.set_fields_from_dict(goal, request_data)

            future = client.send_goal_async(
                goal,
                feedback_callback=lambda m: self.handle_feedback_callback(action_name, m)
            )

            with self.pending_lock:
                self.pending_requests[action_name] = [future, addr]

            future.add_done_callback(
                lambda f: self.handle_response_callback(action_name, f)
            )

        except json.JSONDecodeError as e:
            self.get_logger().error("Failed to parse JSON: " + str(e))
        except Exception as e:
            self.get_logger().error("Failed to process action call: " + str(e))

    def handle_response_callback(self, action_name, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Action f{action_name} rejected goal")
            return

        with self.pending_lock:
            if action_name not in self.pending_requests:
                self.get_logger().warning(
                    f"Response for unknown action {action_name}"
                )
                return

            self.pending_requests[action_name][0] = goal_handle.get_result_async()
            self.pending_requests[action_name][0].add_done_callback(
                lambda f: self.handle_result_callback(action_name, f)
            )

    def handle_result_callback(self, action_name, future):
        with self.pending_lock:
            if action_name not in self.pending_requests:
                self.get_logger().warning(
                    f"Result for unknown action {action_name}"
                )
                return

            _, addr = self.pending_requests.pop(action_name)

        try:
            result = future.result()
            result_dict = self.dict_from_message(result)

            payload = {
                "action": action_name,
                "result": result_dict
            }
            self.send_udp_message(payload)

            self.get_logger().info(
                f"Action result sent for {action_name}"
            )
        except Exception as e:
            self.get_logger().error(f"Action result failed for {action_name}: {e}")

    def handle_feedback_callback(self, action_name, msg):
        with self.pending_lock:
            if action_name not in self.pending_requests:
                self.get_logger().warning(
                    f"Feedback for unknown action {action_name}"
                )
                return

            _, addr = self.pending_requests[action_name]

        try:
            feedback_dict = self.dict_from_message(msg)

            payload = {
                "action": action_name,
                "feedback": feedback_dict
            }
            self.send_udp_message(payload)

            self.get_logger().info(
                f"Feedback sent for {action_name}"
            )
        except Exception as e:
            self.get_logger().error(f"Feedback failed for {action_name}: {e}")

    def send_udp_message(self, message_dict, host="127.0.0.1", port=65440):
        """Send JSON over UDP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            message = json.dumps(message_dict)
            sock.sendto(message.encode("utf-8"), (host, port))
            self.get_logger().info(f"Sent UDP message to {host}:{port}")
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")
        finally:
            sock.close()

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info("Shutting down service bridge...")

        # Cancel pending requests
        with self.pending_lock:
            count = len(self.pending_requests)
            self.pending_requests.clear()

        if count > 0:
            self.get_logger().info(f"Cancelled {count} pending service requests")

        self.udp_server.stop()
        if self.udp_thread.is_alive():
            self.udp_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPBridgeAction()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()