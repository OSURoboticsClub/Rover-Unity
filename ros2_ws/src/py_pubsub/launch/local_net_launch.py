from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = {
        'respawn': True
    }
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='tcp_relay',
            name='tcp_relay',
            **config
        ),
        Node(
            package='py_pubsub',
            executable='udp_relay',
            name='udp_relay',
	    **config
        ),
    ])
