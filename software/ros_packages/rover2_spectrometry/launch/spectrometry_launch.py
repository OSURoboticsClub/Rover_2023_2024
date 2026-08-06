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
            package='rover2_spectrometry',
            executable='spectrometry_publisher',
            name='spectrometry_publisher',
            **config
        ),
    ])
