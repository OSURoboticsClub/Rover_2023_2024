import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PythonExpression



def generate_launch_description():

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    # -------------------------
    # Launch arguments
    # -------------------------
    arm_arg = DeclareLaunchArgument(
        'arm',
        default_value='True',
        description='Launch arm and arm control'
    )

    auton_arg = DeclareLaunchArgument(
        'auton',
        default_value='False',
        description='Launch autonomy stack (nav + mapping)'
    )

    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value='False',
        description='Launch lidar and config mapping for lidar'
    )

    attachment_arg = DeclareLaunchArgument(
        'attachment',
        default_value='arm',
        description='Attachment type for the arm (arm, none)'
    )

    arm = LaunchConfiguration('arm')
    auton = LaunchConfiguration('auton')
    lidar = LaunchConfiguration('lidar')
    attachment = LaunchConfiguration('attachment')
    arm_enabled = PythonExpression([
        "'", arm, "'.lower() == 'true' and ",
        "'", auton, "'.lower() == 'false'"
    ])

    linear_vel_scalar = PythonExpression([
        "0.5 if '", auton, "'.lower() == 'true' else 1.0"
    ])
    angular_vel_scalar = PythonExpression([
        "0.25 if '", auton, "'.lower() == 'true' else 1.0"
    ])

    # -------------------------
    # Core systems (always on)
    # -------------------------
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover2_control'),
                'odrive_ros2_control.launch.py'
            )
        ),
        launch_arguments={
            "launch_ros2_control": "False",
            "linear_vel_scalar": linear_vel_scalar,
            "angular_vel_scalar": angular_vel_scalar,
        }.items()
    )

    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover2_odometry'),
                'rover2_odometry_launch.py'
            )
        )
    )

    status = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_status'),
         'launch'), '/rover2_status_launch.py'])
      )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover2_main"),
        "config",
        "rover_control.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        **config
    )

    robot_description = Command([
        FindExecutable(name="xacro"),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('rover2_arm'),
            'config',
            'rover.urdf.xacro'
        ]),
        ' ',
        TextSubstitution(text='ros2_control_hardware_type:=main'),
        ' ',
        TextSubstitution(text='attachment:='),
        attachment,
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'use_sim_time': False,
            "robot_description": ParameterValue(robot_description, value_type=str)
        }],
        output="screen",
    )

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

    # -------------------------
    # Cameras (always on, but get auton flag)
    # -------------------------
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover2_camera'),
                'launch',
                'camera_capture_launch.py'
            )
        ),
        launch_arguments={
            "d455": auton,
            "d405": arm_enabled,
            "muxing": auton,
            "driver_cams": PythonExpression(["'", auton, "'.lower() == 'false'"])
        }.items()
    )

    # -------------------------
    # Arm stack (conditional)
    # -------------------------
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover2_arm'),
                'launch',
                'rover2_arm.launch.py'
            )
        ),
        condition=IfCondition(arm_enabled),
        launch_arguments={
            "launch_ros2_control": "False",
            "attachment": attachment,
        }.items()
    )

    arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover_arm_control'),
                'rover_arm_control.launch.py'
            )
        ),
        condition=IfCondition(arm_enabled),
    )

    # -------------------------
    # Autonomy stack (conditional)
    # -------------------------
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav_autonomy'),
                'launch',
                'mapping_launch.py'
            )
        ),
        condition=IfCondition(auton)
    )

    nav_autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav_autonomy'),
                'launch',
                'nav_launch.py'
            )
        ),
        condition=IfCondition(auton)
    )

    delay_mapping = TimerAction(
        period=3.0,
        actions=[mapping]
    )

    # -------------------------
    # Other stuff I don't need rn
    # -------------------------
    driller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover2_drill'),
                'launch',
                'drill_launch.py'
            )
        )
    )

    camera_gimbals = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rover2_control'),
                'gimbal_launch.py'
            )
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_lidar_ros2'),
                'launch.py'
            )
        ),
        condition=IfCondition(lidar)
    )

    # -------------------------
    # Launch description
    # -------------------------
    return LaunchDescription([
        TimerAction(            # Delay so autolaunch works
            period=3.0,
            actions=[
                arm_arg,
                auton_arg,
                lidar_arg,
                attachment_arg,

                ros2_control_node,
                robot_state_publisher_node,
                joint_state_broadcaster_spawner,

                control_launch,
                odom,
                status,
               # cameras,
                lidar,

                arm_launch,
                arm_control,

                delay_mapping,
                #nav_autonomy,

                # camera_gimbals,
                # driller,
            ]
        )
    ])
