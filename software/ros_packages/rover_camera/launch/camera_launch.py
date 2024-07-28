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
            package='rover_camera',
            executable='rover_camera',
            name='navigation',
            parameters=[{
                'device_path': '/dev/rover/camera_main_navigation',
                'base_topic': 'cameras/main_navigation'
            }],
            #prefix=["sudo taskset -c 3"],
            **config
        ),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='chassis',
            #prefix=["sudo taskset -c 4"],
            parameters=[{
                'device_path': '/dev/rover/camera_chassis',
                'base_topic': 'cameras/chassis'
            }],
            **config
        )
    ])
