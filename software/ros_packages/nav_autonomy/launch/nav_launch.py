import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    #ros2_control = LaunchConfiguration('ros2_control', default='true') 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    viz = LaunchConfiguration('rtab_viz', default='true')

    package_name='nav_autonomy'
    pkg_share = get_package_share_directory(package_name)

    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')


    rviz_config = os.path.join(pkg_share, 'rviz', 'depth_nav.rviz')

    # TODO: Maybe replace the current drive controller with ros2_control and diff_drive_controller
    # diff_drive_controller_spawner = Node(
    #     package='controller_manager', executable='spawner',
    #     arguments=['diff_drive_controller'],
    #     #remappings=[('diff_drive_controller/odom', '/odom')],
    #     output='screen'
    #     # ros2 run topic_tools relay /diff_drive_controller/odom /odom
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager', executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen'
    # )
    
    # joystick = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(
    #                     get_package_share_directory(package_name),'launch','joystick.launch.py'
    #         )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )
   
    # twist_mux = Node(
    #         package='twist_mux',
    #         executable='twist_mux',
    #         parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel', '/command_control/ground_station_drive')]
    # )

    
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        # DeclareLaunchArgument('ros2_control', default_value='true', description='Use ros2_control'),

        # rsp,
        # joint_state_broadcaster_spawner, 
        # diff_drive_controller_spawner
        # joystick,
    #    twist_mux,
        # rtabmap_point_cloud,
        # rtabmap_obstacles,
        nav2,
    ])
