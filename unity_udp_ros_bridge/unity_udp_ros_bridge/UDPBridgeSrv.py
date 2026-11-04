import rclpy
from rclpy.node import Node
from unity_udp_ros_bridge.UDPServer import UDPServer
from threading import Thread, Lock
from rosidl_runtime_py.utilities import get_service
import json
import uuid
import socket


class UDPBridgeSrv(Node):
    """ROS2 Node for handling service calls via UDP"""
    
    def __init__(self):
        super().__init__('udp_service_bridge')
        
        # Cache for service type classes
        self.service_type_cache = {}
        
        # Cache for service clients
        self.client_cache = {}
        
        # Track pending service calls: request_id -> (future, addr)
        self.pending_requests = {}
        self.pending_lock = Lock()
        
        # Create UDP server instance
        self.udp_server = UDPServer('127.0.0.1', 65437, logger=self.get_logger())
        
        # Start server in background thread
        self.udp_thread = Thread(
            target=self.udp_server.start, 
            args=(self.process_udp_message,)
        )
        self.udp_thread.daemon = True
        self.udp_thread.start()
        
        self.get_logger().info("UDPServiceBridge node initialized on port 65437")
    
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
            elif isinstance(value, list):
                result[field] = [
                    self.dict_from_message(item) if hasattr(item, '__slots__') else item
                    for item in value
                ]
            else:
                result[field] = value
        return result
    
    def process_udp_message(self, data, addr):
        """
        Handle incoming UDP service call requests
        
        Expected JSON format:
        {
            "service": "/service_name",
            "srvType": "std_srvs/srv/Empty",
            "request": {},
            "response": {}
        }
        """
        try:
            message = data.decode('utf-8').strip()
            payload = json.loads(message)
            
            service_name = payload.get("service")
            service_type = payload.get("srvType")
            request_data = payload.get("request", {})
            response_data = payload.get("response", {})
            # Generate request_id if not provided            
            if not service_name or not service_type:
                self.get_logger().error(
                    "JSON missing required keys 'service' or 'srvType'"
                )
                self.send_error_response(addr, request_id, "Missing required fields")
                return
            
            # Get or create service type class
            if service_type not in self.service_type_cache:
                self.service_type_cache[service_type] = get_service(service_type)
            
            srv_class = self.service_type_cache[service_type]
            
            # Get or create service client
            client_key = f"{service_name}:{service_type}"
            if client_key not in self.client_cache:
                self.client_cache[client_key] = self.create_client(
                    srv_class, 
                    service_name
                )
                self.get_logger().info(
                    f"Created service client for {service_name} with type {service_type}"
                )
            
            client = self.client_cache[client_key]
            
            # Wait for service availability
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f"Service {service_name} not available")
                
                return
            
            # Create and populate request
            request = srv_class.Request()
            self.set_fields_from_dict(request, request_data)
            
            # Send async service call
            future = client.call_async(request)
            
            # Store pending request
            with self.pending_lock:
                self.pending_requests[service_name] = (future, addr)
            
            # Add callback for when response is received
            future.add_done_callback(
                lambda f: self.handle_service_response(service_name, f)
            )
            
            self.get_logger().info(
                f"Service call sent to {service_name}"
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to process service call: {e}")
            if 'request_id' in locals():
                self.send_error_response(addr, request_id, str(e))
    
    def handle_service_response(self, service_name, future):
        """Callback when service response is received"""
        with self.pending_lock:
            if service_name not in self.pending_requests:
                self.get_logger().warning(
                    f"Response for unknown service_name: {service_name}"
                )
                return
            
            _, addr = self.pending_requests.pop(service_name)
        
        try:
            response = future.result()
            response_dict = self.dict_from_message(response)
            
            # Send success response back via UDP
            self.send_success_response(addr, service_name, response_dict)
            
            self.get_logger().info(
                f"Service response sent for service_name {service_name}"
            )
            
        except Exception as e:
            self.get_logger().error(
                f"Service call failed for service_name {service_name}: {e}"
            )
            self.send_error_response(addr, service_name, str(e))
    
    def send_success_response(self, addr, service_name, response_data):
        """Send successful service response via UDP"""
        payload = {
            "service": service_name,
            "response": response_data
        }
        self.send_udp_message(payload)
    
    def send_error_response(self, addr, service_name, error_message):
        """Send error response via UDP"""
        payload = {
            "service": service_name,
            "response": error_message
        }
        self.send_udp_message(payload)
    

    def send_udp_message(self, message_dict, host="127.0.0.1", port=65438):
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
    node = UDPBridgeSrv()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()