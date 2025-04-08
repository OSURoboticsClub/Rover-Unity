import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/makemorerobot/Rover-Unity/ros2_ws/install/AutonControlNode'
