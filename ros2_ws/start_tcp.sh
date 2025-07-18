#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/Rover-Unity/ros2_ws
. install/setup.bash
ros2 run py_pubsub tcp_relay
