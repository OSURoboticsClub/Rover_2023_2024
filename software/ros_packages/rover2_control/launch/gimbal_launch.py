from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'rover2_control_interface',
            namespace = 'tower_gimbal',
            executable = 'odrive_pan_tilt',
            name = 'tower_gimbal',
            parameters = [
                {"control_topic": "/tower_gimbal/control"},
                {"can", "can0"},
                {"direction": 1},
                {"node_ids": [0, 1, 2]},
                {"board_id": 0}
            ]
        ),
        Node (
            package = 'rover2_control_interface',
            namespace = 'chassis_gimbal',
            executable = 'odrive_pan_tilt',
            name = 'chassis_gimbal',
            parameters = [
                {"control_topic": "/chassis_gimbal/control"},
                {"can", "can0"},
                {"direction": 1},
                {"node_ids": [0, -1, 1]},
                {"board_id": 1}
            ]
        )
    ])