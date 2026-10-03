#!/bin/bash

source ./ros2_ws/install/setup.bash
source ./unity_udp_ros_bridge/install/setup.bash
source ../Rover_2023_2024/software/install/setup.bash

ros2 launch py_pubsub local_net_launch.py &
ros2 launch unity_udp_ros_bridge udp_bridge_launch.py &
