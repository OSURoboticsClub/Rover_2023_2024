import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('nav_use_sim_time', default='false') 

    package_name='nav_autonomy'
    pkg_share = get_package_share_directory(package_name)

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_obstacles.yaml')

    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_robot_state_pub': 'false',
            'params_file': nav2_params
        }.items()
    )

    mission_manager = Node(
            package='nav_autonomy',
            executable='mission_manager',
    )

    yolo_server = Node(
            package='nav_autonomy',
            executable='yolo_server',
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav_use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        nav2,
        mission_manager,
        yolo_server,
    ])
