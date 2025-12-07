#!/usr/env/bin python3

#Launch file to just start the d405 gripper depth camera

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        #Launch the d405 node:
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d405',
            parameters=[{
                "camera_name": "d405",
                "depth_width": 1280,
                "depth_height": 720,
                "color_width": 1280,
                "color_height": 720,
                "pointcloud.enable": True,
                "align_depth.enable": True,
                "depth_fps": 10,
                "rgb_fps": 10

            }],
            output='screen'
        )
    ])
