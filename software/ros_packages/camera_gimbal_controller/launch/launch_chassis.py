from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'camera_gimbal_controller',
            namespace = 'chassis_gimbal',
            executable = 'chassis_controller',
            name = 'chassis_gimbal_controller',
            parameters = [
                {"gimbal_topic": "/chassis_gimbal/control"}
            ]
        )
    ])