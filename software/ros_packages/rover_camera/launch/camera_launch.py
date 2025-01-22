from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True,
        'respawn_delay': 2
    }

    return LaunchDescription([
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='navigation',
            parameters=[{
                'device_path': '/dev/rover/camera_main_navigation',
                'base_topic': 'cameras/main_navigation',
                'fps': 10
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
        ),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='infrared',
            #prefix=["sudo taskset -c 5"],
            parameters=[{
                'device_path': '/dev/rover/camera_infrared',
                'base_topic': 'cameras/infrared'
            }],
            **config
        ),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='gripper',
            #prefix=["sudo taskset -c 6"],
            parameters=[{
                'is_rtsp_camera': True,
                'device_path': 'rtsp://192.168.1.11:554',
                'base_topic': 'cameras/gripper'
            }],
            **config
        )
    ])
