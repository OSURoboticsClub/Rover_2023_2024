from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'rover2_control',
            namespace = 'tower_gimbal',
            executable = 'odrive_pan_tilt',
            name = 'tower_gimbal',
            parameters = [
                {"control_topic": "/tower_gimbal/control"},
                {"can", "can_cam"},
                {"direction": 1},
                {"node_ids": [5, 3, 4]},
                {"board_id": 2}
            ]
        ),
        Node (
            package = 'rover2_control',
            namespace = 'chassis_gimbal',
            executable = 'odrive_pan_tilt',
            name = 'chassis_gimbal',
            parameters = [
                {"control_topic": "/chassis_gimbal/control"},
                {"can", "can_cam"},
                {"direction": 1},
                {"node_ids": [1, -1, 0]},
                {"board_id": 6}
            ]
        )
    ])