"""
Rover Arm Control Launch
DAM Robotics
Authors: Jared Northrop
Year: 2526

This launch file starts the nodes specific to this package for rover operations. 
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }
    
    gripper_can_control_node = Node(
        package='rover_arm_control',
        executable='gripper_control',
        name='arm_gripper_control',
        parameters=[{
            'is_position_control': False,
            'joy_publish_rate': 50,
            'can': "can_arm"
        }],
        **config
    )
    relative_move_node = Node(
        package='rover_arm_control',
        executable='relative_move',
        name='relative_move',
        **config
    )
    absolute_move_node = Node(
        package='rover_arm_control',
        executable='absolute_move',
        name='absolute_move',
        **config
    )

    #Launch the pc_filter node
    pc_filter_node = Node(
			package = 'pc_processing',
			executable = 'pc_filter',
			name = 'pc_filter',
			remappings = [
				#('/raw_point_cloud', '/astra_ros/devices/default/point_cloud') #Remap for old HW rosbag
				('/raw_point_cloud','/camera/d405/depth/color/points') #Remap for rover d405 pointcloud
			]
    )
    #Launch the plane fitting node
    pc_plane_node = Node(
			package = 'pc_processing',
			executable = 'plane_fit',
			name = 'plane_fit',
    )
    move_perp = Node(
                        package = 'rover_arm_control',
                        executable = 'move_perp_to_plane',
                        name = 'move_perp_to_plane',
    )

    joint_position_controler = Node(
        package='rover_arm_control',
        executable='joint_position_control',
        name='joint_position',
        **config
    )

    return LaunchDescription([
        gripper_can_control_node,
        relative_move_node,
        absolute_move_node,
        pc_filter_node,
        pc_plane_node,
        move_perp,
        joint_position_controler,
    ])
