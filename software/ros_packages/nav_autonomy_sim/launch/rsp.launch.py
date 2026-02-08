import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

###################################
# This file launches a robot state publisher to publish tfs
# Don't need
# On the rover this is handled by the rover_arm (it should probably be separate)
###################################

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    ros2_control = LaunchConfiguration('ros2_control')

    pkg_path = os.path.join(get_package_share_directory('nav_autonomy_sim'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' ros2_control:=', ros2_control])

    params = {
        'robot_description': robot_description_config, 
        'use_sim_time': use_sim_time
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time'),
        DeclareLaunchArgument('ros2_control', default_value='true', description='Use ros2_control'),

        node_robot_state_publisher
    ])
