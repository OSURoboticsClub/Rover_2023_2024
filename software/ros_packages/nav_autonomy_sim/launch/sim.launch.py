import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


####################
# This file launches the current sim setup
# Usage: Sim only
# Can reference for rover integration
####################

def generate_launch_description():

    ros2_control = LaunchConfiguration('ros2_control', default='true') 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 

    package_name='nav_autonomy_sim'
    pkg_share = get_package_share_directory(package_name)

    gazebo_params = os.path.join(pkg_share, 'config', 'gazebo_params.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'rgbd_nav2_params.yaml')
    #rtabmap_params = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')
    rtabmap_params = {
        'use_sim_time': use_sim_time,
        'frame_id':'base_link',
        #'publish_tf': True,
        #'odometry_frame_id': 'odom',
        'subscribe_depth': True,
        #'subscribe_rgbd': True,
        #'subscribe_scan': True,
        'use_action_for_goal': True,
        'approx_sync': True,
        #'Rtabmap/PublishGrid': 'true',
        #'RGBD/PublishCloud': 'true',
        'Reg/Force3DoF': 'true',
        #'Grid/FromDepth': 'true',
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

    rviz_config = os.path.join(pkg_share, 'rviz', 'depth_nav.rviz')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share,'launch','rsp.launch.py')
        ]), 
        launch_arguments={
            'use_sim_time': 'true', 
            'ros2_control': ros2_control
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': f'--ros-args --params-file {gazebo_params}'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', 
        arguments=['-topic', 'robot_description', '-entity', 'auto_bot'], 
        output='screen'
    )

   
    diff_drive_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller'],
        #remappings=[('diff_drive_controller/odom', '/odom')],
        output='screen'
        # ros2 run topic_tools relay /diff_drive_controller/odom /odom
    )
    #delayed_ddc_spawner = RegisterEventHandler(
    #    event_handler=OnProcessExit(
    #        target_action=spawn_entity,
    #        on_exit=[diff_drive_controller_spawner]
    #    )
    #)

    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    #delayed_jsb_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    #)
    
    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', 
        name='rtabmap',
        parameters=[rtabmap_params],
        remappings=rtabmap_remaps,
        arguments=['-d'],
        output='screen',
    )

    rtabmnap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry',
        name='rgbd_odometry',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'wait_imu_to_init': True,
            #'Odom/ResetCountdown': 1,
        }],
        remappings=rtabmap_remaps,
        output='screen',
    )

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz',
        parameters=[rtabmap_params],
        remappings=rtabmap_remaps,
        output='screen',
    )

    # Obstacle detection with the camera for nav2 local costmap.
    # First, we need to convert depth image to a point cloud.
    # Second, we segment the floor from the obstacles.
    rtabmap_point_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                     'max_depth': 10.0,
                     'voxel_size': 0.02}],
        remappings=[('depth/image', '/camera_depth/depth/image_raw'),
                    ('depth/camera_info', '/camera_depth/depth/camera_info'),
                    ('cloud', '/camera/cloud')])
    
    rtabmap_obstacles = Node(
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=[rtabmap_params],
        remappings=[('cloud', '/camera/cloud'),
                    ('obstacles', '/camera/obstacles'),
                    ('ground', '/camera/ground')])


    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory(package_name),'launch','joystick.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

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

    spawn_controllers_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, diff_drive_controller_spawner]
        )
    )

    spawn_slam_nav_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                # localization,
                rtabmap_slam,
                rtabmnap_odom, 
                rtabmap_point_cloud,
                rtabmap_obstacles,
                nav2
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('ros2_control', default_value='true', description='Use ros2_control'),

        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        joystick,
        spawn_controllers_after_entity,
        spawn_slam_nav_after_entity,
        rtabmap_viz,

        #diff_drive_controller_spawner,
        #joint_state_broadcaster_spawner,
        #delayed_ddc_spawner,
        #delayed_jsb_spawner,
        #rtabmap_slam,
        #rtabmap_localization,
        #rtabmap_point_cloud,
        #rtabmap_obstacles
    ])
