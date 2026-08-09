from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    return LaunchDescription([
        Node(
            package='spectrometer_ui',
            executable='spectrometer_control_ui',
            name='spectrometer_control_ui',
            **config
        )
    ])
