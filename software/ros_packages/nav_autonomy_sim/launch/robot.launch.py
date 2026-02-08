import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

##########################
# This file controls the robots movement based on the commands from nav2 stack
# Usage: Sim only (for now)
# Rover currently has another control method, but maybe this is better
##########################

def generate_launch_description():

    #ros2_control = LaunchConfiguration('ros2_control', default='true') 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    viz = LaunchConfiguration('rtab_viz', default='true')

    package_name='nav_autonomy_nav2'
    pkg_share = get_package_share_directory(package_name)

    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'rgbd_nav2_params.yaml')
    rtabmap_params = {
        'use_sim_time': use_sim_time,
        'frame_id':'base_link',
        'map_frame_id': "world",
        'subscribe_odom_info':True,
        'subscribe_depth': True,
        'subscribe_rgbd': True,
        'use_action_for_goal': True,
        'approx_sync': False,
        'Reg/Force3DoF': 'true',
        'Grid/RayTracing': 'true', # Fill empty space
        'Grid/3D': 'false', # Use 2D occupancy
        'Grid/RangeMax': '3',
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight': '0.05', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight': '0.4',  # All points over 1 meter are ignored
        'Optimizer/GravitySigma': '0' # Disable imu constraints (we are already in 2D)
    }
    rtabmap_remaps = [
        ('rgb/image', '/camera_depth/image_raw'),
        ('rgb/camera_info', '/camera_depth/depth/camera_info'),
        ('depth/image', '/camera_depth/depth/image_raw'),
    ]

    # remappings=[
    #     ('rgb/image', '/camera/d455/color/image_raw'),
    #     ('rgb/camera_info', '/camera/d455/color/camera_info'),
    #     ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw')]

    rviz_config = os.path.join(pkg_share, 'rviz', 'depth_nav.rviz')

    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(pkg_share,'launch','rsp.launch.py')
    #     ]), 
    #     launch_arguments={
    #         'use_sim_time': 'true', 
    #         'ros2_control': ros2_control
    #     }.items()
    # )

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
    
    rtabmnap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry',
        name='rgbd_odometry',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 'approx_sync': True,
            # 'wait_imu_to_init': True,
            #'Odom/ResetCountdown': 1,
        }],
        remappings=rtabmap_remaps,
        output='screen',
    )

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', 
        name='rtabmap',
        parameters=[rtabmap_params],
        remappings=rtabmap_remaps,
        arguments=['-d'],
        output='screen',
    )

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz',
        parameters=[rtabmap_params],
        remappings=rtabmap_remaps,
        output='screen',
        condition=IfCondition(viz)
    )

    # Obstacle detection with the camera for nav2 local costmap.
    # First, we need to convert depth image to a point cloud.
    # Second, we segment the floor from the obstacles.
    # rtabmap_point_cloud = Node(
    #     package='rtabmap_util', executable='point_cloud_xyz', output='screen',
    #     parameters=[{'decimation': 2,
    #                  'max_depth': 10.0,
    #                  'voxel_size': 0.02}],
    #     remappings=[('depth/image', '/camera_depth/depth/image_raw'),
    #                 ('depth/camera_info', '/camera_depth/depth/camera_info'),
    #                 ('cloud', '/camera/cloud')])
    #
    # rtabmap_obstacles = Node(
    #     package='rtabmap_util', executable='obstacles_detection', output='screen',
    #     parameters=[rtabmap_params],
    #     remappings=[('cloud', '/camera/cloud'),
    #                 ('obstacles', '/camera/obstacles'),
    #                 ('ground', '/camera/ground')])


    # joystick = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([os.path.join(
    #                     get_package_share_directory(package_name),'launch','joystick.launch.py'
    #         )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )
    #
    # twist_mux = Node(
    #         package='twist_mux',
    #         executable='twist_mux',
    #         parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    # )

    localization = Node(
                         package='robot_localization',
                         executable='ekf_node',
                         name='ekf_global',
                         output='screen',
                         parameters=[{
                                     'frequency': 50.0,
                                     'sensor_timeout': 0.1,
                                     'two_d_mode': True,
                                     'transform_time_offset': 0.0,
                                     'transform_timeout': 0.0,
                                     'print_diagnostics': True,
                                     'debug': False,
                                     'publish_tf': True,
    
                                     'map_frame': 'map',
                                     'odom_frame': 'odom',
                                     'base_link_frame': 'base_link',
                                     'world_frame': 'odom', 
    
                                     # Local odometry 
                                     'odom0': '/odom',
                                     'odom0_config': [False, False, False,
                                                      False, False, False,
                                                      True,  True,  False,
                                                      False, False, True,
                                                      False, False, False],
                                     'odom0_queue_size': 10,
                                     'odom0_differential': False,
                                     'odom0_relative': False,
    
                                     # IMU (same as local)
                                     'imu0': '/imu',
                                     'imu0_config': [False, False, False,
                                                     False, False, True,
                                                     False, False, False,
                                                     False, False, True,
                                                     True,  True,  False],
                                     'imu0_queue_size': 10,
                                     'imu0_differential': False,
                                     'imu0_relative': False,
                                     'imu0_remove_gravitational_acceleration': True,
                         }],
                         remappings=[('/odometry/filtered', '/odometry/global')]
     )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
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
        # twist_mux,
        localization,
        rtabmap_slam,
        rtabmnap_odom, 
        # rtabmap_point_cloud,
        # rtabmap_obstacles,
        nav2,
        rtabmap_viz
    ])
