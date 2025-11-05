# Rover-Unity

## Groundstation Setup

### Getting Started with Groundstation Code

The groundstation code is divided into three main parts:

- **Unity controls and readouts**
- **Connection interface between Unity and ROS2**
- **Data handling** — structured CSV data sent over UDP ports that follows a ROS-like encode/decode format.

---
All setup has been verified working on a fresh installation of Ubuntu 22.04
### Required Software

- **Unity (2022.3.32f1)**  
  [https://docs.unity3d.com/hub/manual/InstallHub.html](https://docs.unity3d.com/hub/manual/InstallHub.html)

- **ROS2 Humble**  
  [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)

- **GStreamer**  
  [https://gstreamer.freedesktop.org/documentation/installing/index.html?gi-language=c](https://gstreamer.freedesktop.org/documentation/installing/index.html?gi-language=c)

- **Colcon**  
  [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

---

### Unity

Fairly standard Unity setup.  
The main difference is the tie-in with ROS, which uses structured CSV data sent over UDP ports that emulate ROS message structures.

There are multiple Unity pages, each representing a different data stream:

- **Cameras**
- **Telemetry**
- **Autonomy**

> **Note:** This system is intended to be used by at least **two laptops** (for ease of use), though one can handle everything if needed.

---

## Install Guide

> **Note:**  
> There is both a **Unity** and **ROS2** aspect to this project.  
> Once ROS2 and Unity are installed, you must clone **this repository** and the **rover_2023_2024** repository to get the bridge between them working.

### Shell Sourcing Reminder

All ROS2 sourcing must be done in the **same shell (terminal)** session.  
You’ll source ROS2 and run `setup.sh` scripts — make sure these steps happen in the same terminal.
Building only has to occur if you ever change any code,.

To make the process easier, consider adding the relevant sourcing commands to your `~/.bashrc`.

---

## Unity ROS2 Repository Setup
### Clone this repository
git clone https://github.com/OSURoboticsClub/Rover-Unity.git

### Navigate to the workspace
cd Rover-Unity/ros2_ws

### Build the py_pubsub package
colcon build --packages-select py_pubsub

### Source the local setup
. install/setup.bash

### Install pip requirements (if necessary)
pip install -r requirements.txt (Not pruned yet, could hold eroneous python packages)


## Rover ROS2 Repository Setup (required for message definitions)

## Clone the rover repository
git clone https://github.com/OSURoboticsClub/Rover_2023_2024.git

## Navigate to the workspace
cd Rover_2023_2024/software

## Build the py_pubsub package
colcon build --packages-select rover2_control_interface rover2_status_interface

## Source the local setup
. install/setup.bash


## How to run
### Unity
In Unity hub, add the whole Rover-Unity directory as a new project from your disk. Compilation takes a while.

Once completed, and the editor is open, double click the "Application" file in your bottom file explorer.

### ROS Bridge
cd Rover-Unity/ros2_ws

ros2 launch py_pubsub local_net_launch.py






