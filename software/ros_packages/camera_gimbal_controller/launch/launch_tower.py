from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'camera_gimbal_controller',
            namespace = 'tower_gimbal',
            executable = 'tower_controller',
            name = 'tower_gimbal_controller',
            parameters = [
                {"gimbal_topic": "/tower_gimbal/control"}
            ]
        )
    ])