# Rover-Unity
## Groundstation Setup

### Getting Started With Groundstation Code
Groundstation code can largely be split into 3 sections - Pure unity controls and readouts, and the connection interface between Unity and Ros2.

Unity - https://docs.unity3d.com/hub/manual/InstallHub.html (specifically version 2022.3.32f1)

ROS2 Humble - https://docs.ros.org/en/humble/index.html

gstreamer - https://gstreamer.freedesktop.org/documentation/installing/index.html?gi-language=c

#### Unity 
Fairly standard unity stuff. The only interesting part is the tie in with ROS, which is just structured CSV data sent over UDP ports that is strucutred ROS-like for each encode/decode.

There are multiple pages, each representing a data stream - cams, telemetry, autonomy, etc.

This is intended to be used by a minimum of 2 laptops for ease of use, but 1 can do everything.

#### Miscellaneous
NOTE: There is a Unity and ROS aspect to this project. Once ROS2 and Unity are installed, to get the bridge between the 2 working, you must:

install colcon - https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

build in the Rover-Unity/ros2_ws directory 

run . install/setup.bash, and run 

ros2 launch py_pubsub local_net_launch.py - actually launches the bridge

## Rover Setup
This repo does not contain rover code. To view rover code and setup, see [https://github.com/OSURoboticsClub/Rover-Unity]
