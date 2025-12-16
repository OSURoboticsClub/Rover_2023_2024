from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
        package='rover2_control',
        executable='gripper_control',
        name='gripper_can_control',
        parameters=[{
            'is_position_control': False,
            'joy_publish_rate': 50,
            'can': "can1"
        }],
        **config
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output='screen',
        respawn= True

    )


    return LaunchDescription([
        gripper_can_control_node,
        joy_node
    ])