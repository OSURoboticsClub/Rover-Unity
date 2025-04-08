#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import gi
import sys
import time
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading

class GstreamerImagePublisher(Node):
    def __init__(self):
        super().__init__('gstreamer_image_publisher')
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Create CV Bridge
        self.bridge = CvBridge()
        
        # Create publisher
        self.declare_parameter('output_topic', 'camera/image_raw')
        self.declare_parameter('udp_port', 5000)
        self.declare_parameter('buffer_size', 2)
        self.declare_parameter('queue_size', 10)
        
        output_topic = self.get_parameter('output_topic').value
        udp_port = self.get_parameter('udp_port').value
        buffer_size = self.get_parameter('buffer_size').value
        queue_size = self.get_parameter('queue_size').value
        
        self.image_pub = self.create_publisher(Image, output_topic, queue_size)
        
        # Create optimized pipeline
        pipeline_str = (
            f"udpsrc port={udp_port} buffer-size=65536 ! "
            "application/x-rtp, payload=96 ! "
            f"rtpjitterbuffer latency=50 ! " 
            "rtph264depay ! "
            "h264parse ! "
            "avdec_h264 max-threads=4 ! "
            f"queue max-size-buffers={buffer_size} leaky=downstream ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=True sync=false max-buffers=1 drop=true"
        )
        
        self.get_logger().info(f"Pipeline: {pipeline_str}")
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        
        # Connect to the new-sample signal
        self.sink.connect("new-sample", self.on_new_sample)
        
        # Create a GLib MainLoop and run it in a separate thread
        self.loop = GLib.MainLoop()
        self.thread = threading.Thread(target=self.loop.run)
        self.thread.daemon = True
        
        # For monitoring frame rate
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.fps = 0
        
        # Start publishing
        self.get_logger().info('Starting GStreamer image publisher')
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set pipeline to playing state!")
            sys.exit(1)
            
        self.thread.start()
        
        # Timer to check status and report FPS
        self.timer = self.create_timer(5.0, self.check_status)
    
    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR
        
        # Get buffer from sample
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        
        # Extract width and height info from caps
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        
        # Extract data from buffer
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            buffer.unmap(map_info)
            return Gst.FlowReturn.ERROR
        
        try:
            # Create numpy array from buffer data
            img_array = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            )
            
            # Create ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(img_array, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            # Publish image
            self.image_pub.publish(ros_image)
            
            # Calculate FPS
            current_time = time.time()
            self.frame_count += 1
            time_diff = current_time - self.last_frame_time
            if time_diff >= 1.0:
                self.fps = self.frame_count / time_diff
                self.frame_count = 0
                self.last_frame_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {str(e)}")
        finally:
            # Clean up
            buffer.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def check_status(self):
        state = self.pipeline.get_state(0)[1]
        if state != Gst.State.PLAYING:
            self.get_logger().warning(f"Pipeline not in PLAYING state! Current state: {state}")
            self.get_logger().info("Attempting to restart pipeline...")
            self.pipeline.set_state(Gst.State.PLAYING)
        else:
            self.get_logger().info(f"Pipeline running at {self.fps:.2f} FPS")
    
    def destroy_node(self):
        self.get_logger().info('Shutting down GStreamer pipeline')
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
        self.thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GstreamerImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
