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
            parameters=[{
                'camera_locations': '/dev/rover/spectrometer_cam_',
                'timer_period_s': .1,
                # Wait time between image sends in seconds
                'image_wait_time': 30,
            }],
            **config
        ), # Local System - /dev/video
        Node(
            package='rover2_spectrometry',
            executable='spectrometry_mechanical',
            name='spectrometry_mechanical',
            parameters=[{
                "can_bus": "can0",
                "node_id": 60,
                "timer_period_s": .02,
            }],
            **config
        ),
    ])
