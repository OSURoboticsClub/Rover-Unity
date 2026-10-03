#!/usr/bin/env python3
"""
Capture a single frame from an H265 RTP UDP GStreamer stream and save it to disk.

This script connects to a stream like the one you showed (port 42069) and saves
one frame then exits.

Example:
  python3 gripper_cap.py --port 42069 --out frame.jpg

Dependencies: OpenCV with GStreamer support (cv2). On Debian/Ubuntu:
  sudo apt install python3-opencv gstreamer1.0-plugins-{base,good,bad,ugly} gstreamer1.0-libav
"""

import argparse
import sys
import time
from typing import List

try:
	import cv2
except Exception as e:
	print("Error: OpenCV (cv2) is required and must be built with GStreamer support.", file=sys.stderr)
	print(e, file=sys.stderr)
	sys.exit(2)


def make_gst_pipeline(port: int) -> str:
	# Build a GStreamer pipeline that decodes H265 RTP over UDP and exposes BGR frames to appsink
	return (
		f'udpsrc port={port} caps="application/x-rtp, media=video, encoding-name=H265, payload=96" ! '
		'rtpjitterbuffer latency=200 ! '
		'rtpulpfecdec ! '
		'rtph265depay ! '
		'h265parse ! '
		'queue max-size-buffers=3000 max-size-time=0 max-size-bytes=0 ! '
		'avdec_h265 ! '
		'videoconvert ! '
		'videorate ! '
		'video/x-raw,format=BGR,framerate=30/1 ! '
		'appsink max-buffers=1 drop=false sync=false'
	)


def capture_one_frame(port: int, timeout: float) -> "tuple[bool, object]":
	pipeline = make_gst_pipeline(port)
	cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
	if not cap.isOpened():
		return False, f"failed to open GStreamer pipeline on port {port}"

	start = time.time()
	frames_read = 0
	while True:
		ret, frame = cap.read()
		if ret and frame is not None:
			frames_read += 1
			# allow decoder to warm up by skipping the first frame or two
			if frames_read >= 50:
				cap.release()
				return True, frame

		if time.time() - start > timeout:
			cap.release()
			return False, f"timeout ({timeout}s) waiting for frame"

		time.sleep(0.01)


def parse_args():
	p = argparse.ArgumentParser(description="Capture one frame from H265 UDP RTP stream and save it")
	p.add_argument("--port", "-p", type=int, default=42069, help="UDP port to listen on (default: 42069)")
	p.add_argument("--out", "-o", default="frame.jpg", help="Output image file (default: frame.jpg)")
	p.add_argument("--timeout", "-t", type=float, default=10.0, help="Seconds to wait for a frame (default: 10)")
	return p.parse_args()


def write_frame(
		name: str,
		port: int,
		includeHeading: bool = True,
		headingOffset: float = 0.0,
		includeScale: bool = False,
		scalePortion: float = 0.0, 
		includeGNSS: bool = True,
		scaleSize: float = 10.0
	):

	print("Starting Image Capture...")
	ok, result =  capture_one_frame(port, 5.0)
	if not ok:
		print(f"Error: {result}", file=sys.stderr)
		sys.exit(1)

	frame = result

	# Optionally get heading from ROS topic /imu/heading (Float32)
	heading_val = None
	if includeHeading:
		print("Getting Heading...")
		try:
			import rclpy
			from std_msgs.msg import Float32
			# initialize/shutdown only if needed
			initialized_here = False
			if not rclpy.ok():
				rclpy.init()
				initialized_here = True
			node = rclpy.create_node('gsteam_image_capture_heading_reader')
			heading_container = [None]  # use list for mutability in callback
			def _cb(msg):
				heading_container[0] = float(msg.data)
			sub = node.create_subscription(Float32, '/imu/heading', _cb, 10)
			# spin briefly to receive a message
			start = time.time()
			while (time.time() - start < 15) and (heading_container[0] is None):
				rclpy.spin_once(node, timeout_sec=0.05)
			heading_val = heading_container[0]
			print(f"Heading value: {heading_val}")
			node.destroy_node()
			if initialized_here:
				rclpy.shutdown()
		except Exception as e:
			# If ROS not available or fails, continue without heading
			print(f"Heading read error: {e}")
			heading_val = None

	# Optionally get GNSS from ROS topic /gps/fix (NavSatFix)
	gnss_val = None
	if includeGNSS:
		print("Getting GNSS...")
		try:
			import rclpy
			from sensor_msgs.msg import NavSatFix
			initialized_here = False
			if not rclpy.ok():
				rclpy.init()
				initialized_here = True
			node = rclpy.create_node('gsteam_image_capture_gnss_reader')
			gnss_container = {'val': None}
			def _gnss_cb(msg):
				gnss_container['val'] = (float(msg.latitude), float(msg.longitude), float(msg.altitude), msg.position_covariance)
			sub = node.create_subscription(NavSatFix, '/gps/fix', _gnss_cb, 10)
			start = time.time()
			while (time.time() - start < 15) and (gnss_container['val'] is None):
				rclpy.spin_once(node, timeout_sec=0.05)
			gnss_val = gnss_container['val']
			print(f"GNSS value: {gnss_val}")
			node.destroy_node()
			if initialized_here:
				rclpy.shutdown()
		except Exception as e:
			print(f"GNSS read error: {e}")
			gnss_val = None

	# Draw annotations
	h, w = frame.shape[:2]
	center_x = w // 2
	center_y = h // 2

	font = cv2.FONT_HERSHEY_SIMPLEX
	font_scale = max(0.17, min(2.0, w / 1800.0))
	thickness = max(1, int(round(font_scale)))
	text_color = (0, 255, 0)
	bg_color = (0, 0, 0)

	# Draw cross in center
	cross_size = int(min(w, h) * 0.03)
	cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), text_color, 2)
	cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), text_color, 2)

	# Heading text below cross
	if includeHeading:
		heading_text = None
		if heading_val is not None:
			val = heading_val + headingOffset
			# normalize to [-180, 180)
			val = ((val + 180.0) % 360.0) - 180.0
			heading_text = f"Heading: {val:.1f}"
		else:
			heading_text = "Heading: N/A"
		# compute text size and position to center under cross
		(tw, th), _ = cv2.getTextSize(heading_text, font, font_scale, thickness)
		text_x = center_x - tw // 2
		text_y = center_y + cross_size + th + 6
		# draw background rectangle for readability
		cv2.rectangle(frame, (text_x - 4, text_y - th - 4), (text_x + tw + 4, text_y + 4), bg_color, -1)
		cv2.putText(frame, heading_text, (text_x, text_y), font, font_scale, text_color, thickness, cv2.LINE_AA)

	# Scale bar near bottom
	if includeScale and scalePortion > 0.0:
		line_length = int(w * float(scalePortion))
		line_y = int(h - max(20, h * 0.05))
		x1 = center_x - line_length // 2
		x2 = center_x + line_length // 2
		cv2.line(frame, (x1, line_y), (x2, line_y), text_color, max(2, thickness + 1))
		# scale text below line
		scale_text = f"{scaleSize:.1f} cm"
		(stw, sth), _ = cv2.getTextSize(scale_text, font, font_scale, thickness)
		stx = center_x - stw // 2
		sty = line_y + sth + 10
		cv2.rectangle(frame, (stx - 4, sty - sth - 4), (stx + stw + 4, sty + 4), bg_color, -1)
		cv2.putText(frame, scale_text, (stx, sty), font, font_scale, text_color, thickness, cv2.LINE_AA)

	# GNSS overlay in top-right
	if includeGNSS:

		precision = "Precision Unkown"

		if gnss_val is None:
			precision = "No Fix"		
		elif list(gnss_val[3]) == list([
                0.01, 0.0, 0.0, 
                0.0, 0.01, 0.0, 
                0.0, 0.0, 0.04
            ]):
			precision = "Loc. +-0.5 m"
		elif list(gnss_val[3]) == list([
                1.0, 0.0, 0.0, 
                0.0, 1.0, 0.0, 
                0.0, 0.0, 4.0
            ]):
			precision = "Loc. +-2 m"
		elif list(gnss_val[3]) == list([
                5.0, 0.0, 0.0, 
                0.0, 5.0, 0.0, 
                0.0, 0.0, 10.0
            ]):
			precision = "Loc. +-5m"
		elif list(gnss_val[3]) == list([
                1e6, 0.0, 0.0, 
                0.0, 1e6, 0.0, 
                0.0, 0.0, 1e6
            ]):
			precision = "No Fix"

		lines = []
		if gnss_val is not None:
			lat, lon, alt, _ = gnss_val
			lines.append(f"Lat: {lat:.6f}")
			lines.append(f"Lon: {lon:.6f}")
			lines.append(precision)
			# show altitude in meters if available
			try:
				lines.append(f"Alt: {alt:.1f} m")
			except Exception:
				lines.append("Alt: N/A")
		else:
			lines.append("GNSS: N/A")

		# measure text block
		padding = max(6, int(6 * font_scale))
		max_w = 0
		line_h = 0
		for l in lines:
			(wt, ht), _ = cv2.getTextSize(l, font, font_scale, thickness)
			if wt > max_w:
				max_w = wt
			line_h = max(line_h, ht)

		block_w = max_w + padding * 2
		block_h = line_h * len(lines) + padding * 2 + (len(lines) - 1) * int(4 * font_scale)

		# top-right corner coordinates
		margin = max(8, int(8 * font_scale))
		x2 = w - margin
		x1 = x2 - block_w
		y1 = margin
		y2 = y1 + block_h

		# draw background rectangle
		cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), bg_color, -1)

		# draw each line
		for i, l in enumerate(lines):
			tx = int(x1 + padding)
			ty = int(y1 + padding + (i + 1) * line_h + i * int(4 * font_scale))
			cv2.putText(frame, l, (tx, ty), font, font_scale, text_color, thickness, cv2.LINE_AA)

	# Save image
	saved = cv2.imwrite(name, frame)
	if not saved:
		print(f"Failed to save image to {name}", file=sys.stderr)
		sys.exit(1)

	print(f"Saved frame to {name}")

def main():
	args = parse_args()
	ok, result = capture_one_frame(args.port, args.timeout)
	if not ok:
		print(f"Error: {result}", file=sys.stderr)
		sys.exit(1)

	frame = result
	# Save image
	saved = cv2.imwrite(args.out, frame)
	if not saved:
		print(f"Failed to save image to {args.out}", file=sys.stderr)
		sys.exit(1)

	print(f"Saved frame to {args.out}")


if __name__ == "__main__":
	main()

