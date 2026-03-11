from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_name = 'rover_gazebo' 
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim') 
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    rover2_control_path = get_package_share_directory('rover2_control')
    rover_gazebo_path = get_package_share_directory(pkg_name)
    # print(f"ros_gz_sim_pkg_path: {ros_gz_sim_pkg_path}")

    #Gazebo Model Path
    rover_model_path = os.path.join(get_package_share_directory('rover_urdf'), '..')
    world_model_path = os.path.join(get_package_share_directory('rover_gazebo', '..'))

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + rover_model_path 
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + rover_model_path 

    print(f"GZ_SIM_RESOURCE_PATH: {os.environ['GZ_SIM_RESOURCE_PATH']}")

    print(f"rover_model_path: {rover_model_path}")

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
    )

    #Load Robot Description
    robot_description = Command([
        FindExecutable(name="xacro"), 
        ' ',
        PathJoinSubstitution([
            FindPackageShare('rover2_arm'),
            'config',
            'rover.urdf.xacro'
        ]),
        ' ',
        TextSubstitution(text='ros2_control_hardware_type:=gazebo'),
        ' ',
        TextSubstitution(text='attachment:=none'),
    ])

    #robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'use_sim_time': use_sim_time,
            "robot_description": robot_description
        }],
        output="screen",
    )

    #ros2 control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover2_control"),
        "config",
        "rover_drive.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Load joint_state_broadcaster after ros2_control_node is up
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    drive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_controller', "-c", "/controller_manager"],
        output='screen'
    )

    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf2_broadcaster",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "rover_base_origin"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': os.path.join(rover_gazebo_path, 'worlds/rubicon_model.sdf') +' -r -v 1',
            'on_exit_shutdown': 'True'
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover",
            "-allow_renaming",
            "-x", "-8.8", "-y", "-1.96", "-z", "4.5"
        ],
        output="screen"
    )

    joy_to_drive = Node(
        package="rover2_control",
        executable="joy_to_drive",
        name="joy_to_drive",
        output="screen"
    )

    ign_ros_bridge_config = PathJoinSubstitution([
        FindPackageShare("rover_gazebo"),
        "config",
        "ros2_gz_bridge.yaml"
    ])
    gazebo_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": ign_ros_bridge_config,
        }]
    )

    gps_node = Node(
        package="rover_gazebo",
        executable="gps_node",
        name="gps_node",
        output="screen",
    )
    imu_node = Node(
        package="rover_gazebo",
        executable="imu_node",
        name="imu_node",
        output="screen",
    )

    #RViz
    rviz_config_file = (
        get_package_share_directory("rob599_nav_gazebo") + "/config/rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


      # 5. NavSat Transform - converts GPS to map frame
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[{
        'use_sim_time': True,

        # Frequency and timing
        'frequency': 30.0,
        'delay': 3.0,

        # Magnetic declination at your location (radians)
        # Find yours: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        # 'magnetic_declination_radians': 0.253,

        # Use odometry heading instead of IMU
        'use_odometry_yaw': False,
        #'yaw_offset': 1.570796326, # yaw correction of IMU absolute yaw measurement (must point east)

        # 2D navigation
        'zero_altitude': False,

        # Publishing options
        'broadcast_cartesian_transform': True,
        'publish_filtered_gps': True,

        # Let first GPS message set origin
        'wait_for_datum': True,
        #'use_manual_datum': True,
        #'datum': [44.56722346625757, -123.27433385957002, 0.0],

        'base_link_frame_id': 'rover_base_origin',
        'world_frame_id': 'map',  # Match the EKFs
        }],
        remappings=[
        ('/imu', '/imu/data'),                  # IMU topic
        ('/gps/fix', '/gps/fix'),                    # GPS INPUT TOPIC
        ('/gps/filtered', '/gps/filtered'),          # GPS INPUT TOPIC
        ('/odometry/filtered', '/odometry/global'),  # Which EKF to use for heading
        ('/odometry/gps', '/odometry/gps'),          # GPS output topic
        ]
    )

    # Local EKF (odom)
    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'print_diagnostics': True,
            'debug': False,
            'frequency': 30.0,
            'two_d_mode': False,
            'publish_tf': True,
            # 'sensor_timeout': 0.1,
            # 'transform_time_offset': 0.0,
            # 'transform_timeout': 0.0,
            
            # 'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'rover_base_origin',
            'world_frame': 'odom',
            
            # Local odometry 
            'odom0': '/drive_controller/odom',
            'odom0_config': [False, False, False,
                            False, False, False,
                            True,  True,  False,
                            False, False, True,
                            False, False, False],
            'odom0_queue_size': 10,
            'odom0_differential': False,
            'odom0_relative': False,

            # Visual odometry
            # 'odom1': '/odometry/visual',
            # 'odom1_config': [False,  False,  False, # x, y position
            #                 False, False, False,   # yaw orientation
            #                 True, False, False,
            #                 False, False, False,
            #                 False, False, False],
            # 'odom1_queue_size': 10,
            # 'odom1_differential': False,
            # 'odom1_relative': False,
            
            # IMU 
            # (See nav2 gps docs REP 105 odom frame should use only heading from IMU)
            # [false, false, false, false,  false,  true, false, false, false, false,  false,  false, false,  false,  false]
            'imu0': '/imu/data',
            'imu0_config': [False, False, False,
                            False, False, True,
                            False, False, False,
                            False, False, False,
                            False,  False,  False],
            'imu0_queue_size': 10,
            'imu0_differential': False,
            'imu0_relative': True,
            'imu0_remove_gravitational_acceleration': False,
        }],
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
        ]
    )
        
    # Global EKF (map)
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'print_diagnostics': True,
            'debug': False,
            'publish_tf': True,
            'frequency': 10.0,
            'two_d_mode': False,
            # 'sensor_timeout': 0.1,
            # 'transform_time_offset': 0.0,
            # 'transform_timeout': 0.0,
            
            'odom_frame': 'odom',
            'base_link_frame': 'rover_base_origin',
            'world_frame': 'map',
            'map_frame': 'map', # published map frame tf name
            
            # Local odometry 
            'odom0': '/drive_controller/odom',
            'odom0_config': [False, False, False,
                            False, False, False,
                            True,  True,  False,
                            False, False, True,
                            False, False, False],
            'odom0_queue_size': 10,
            'odom0_differential': False,
            'odom0_relative': False,

            # GPS odometry (from navsat_transform) (pose x, y)
            'odom1': 'odometry/gps',
            'odom1_config': [True,  True,  False,   # x, y position
                            False, False, False,
                            False, False, False,
                            False, False, False,
                            False, False, False],
            'odom1_queue_size': 10,
            'odom1_differential': False,
            'odom1_relative': False,
            
            # IMU (accel x, vel yaw)
            'imu0': '/imu/data',
            'imu0_config': [False, False, False,
                            False, False, True,
                            False, False, False,
                            False, False, False,
                            False,  False,  False],
            'imu0_queue_size': 10,
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_remove_gravitational_acceleration': False,
        }],
        remappings=[
            ('/odometry/filtered', 'odometry/global'),
        ]
    )

    params_file = (
        get_package_share_directory("rob599_nav_gazebo") + "/config/default_nav2_params.yaml"
    )

    map_file = (
        get_package_share_directory("rob599_nav_gazebo") + "/config/map.yaml"
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": params_file,
            "slam": "False",
            "use_map_server": "False", 
            "autostart": "True",
            "map":map_file,
        }.items(),
    )

    rtabmap_parameters = {
        'frame_id':'rover_base_origin',
        'use_sim_time':True,
        'subscribe_depth':False,
        'subscribe_scan_cloud': True,
        'use_action_for_goal':True,
        'Reg/Force3DoF':'true',
        'Grid/CellSize': "0.05",  # Voxel downsampling
        'Grid/RayTracing':'true', # Fill empty space
        'Grid/Sensor':'0',
        'Grid/FromDepth':'False',
        'Grid/3D': 'true', # Use 2D occupancy
        'Grid/RangeMax':'5',
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.2', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'1.5',  # All points over 1 meter are ignored
        'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
        'RGBD/CreateOccupancyGrid':"True",
        'RGBD/DepthDecimationr': "4",  # Reduces the depth image resolution before generating the point cloud
        'RGBD/DepthMax': "3.0",  # Filter the depth image
        "delete_db_on_start": True,
    }
    rtabmap_remappings = [
        ('odom', '/odometry/local'),
        ('rgb/image', '/camera/d455/color/image'),
        ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('scan_cloud', '/camera/d455/depth/color/points')
    ]


    rtab_map_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        output="screen"
    )

    rtabmap_vis_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings
    )

    rtabmap_obj_node = Node(
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=[rtabmap_parameters],
        remappings=[('cloud', '/camera/d455/depth/color/points'),
                    ('obstacles', '/camera/obstacles'),
                    ('ground', '/camera/ground')]
    )
    

    # world_path = os.path.join(rover_gazebo_path, 'worlds/rubicon.sdf')   

    return LaunchDescription([
        use_sim_time_arg,
        gazebo_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[drive_controller_node],
            )
        ),
        joy_node,
        gazebo_bridge_node,
        spawn_robot,
        joy_to_drive,
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[rviz_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[navsat_transform_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[gps_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[imu_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[local_ekf_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[global_ekf_node],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[rtab_map_node],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[rtabmap_obj_node],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[rtabmap_vis_node],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[nav2],
            )
        ),


    ])
