from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unity_udp_ros_bridge',
            executable='udp_bridge_pub',
            name='udp_bridge_pub'
        ),
        Node(
            package='unity_udp_ros_bridge',
            executable='udp_bridge_sub',
            name='udp_bridge_sub'
        ),
        Node(
            package='unity_udp_ros_bridge',
            executable='udp_bridge_srv',
            name='udp_bridge_srv'
        ),
        Node(
            package='unity_udp_ros_bridge',
            executable='udp_bridge_action',
            name='udp_bridge_action'
        )
    ])
