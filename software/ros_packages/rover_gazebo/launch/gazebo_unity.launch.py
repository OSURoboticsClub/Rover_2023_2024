from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = "rover_gazebo"
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"])
    rover_gazebo_path = get_package_share_directory(pkg_name)

    model_paths = [
        os.path.join(get_package_share_directory("rover_urdf"), ".."),
        os.path.join(get_package_share_directory("rover_arm_urdf"), ".."),
    ]
    model_path_env = os.pathsep.join(model_paths)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + model_path_env
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "") + os.pathsep + model_path_env

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    attachment = LaunchConfiguration("attachment", default="arm")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    attachment_arg = DeclareLaunchArgument(
        "attachment",
        default_value="arm",
        description="Front attachment to load on the rover model",
    )

    controller_config_path = PathJoinSubstitution([
        FindPackageShare("rover2_main"),
        "config",
        "rover_control.yaml",
    ])

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("rover2_arm"),
            "config",
            "rover.urdf.xacro",
        ]),
        " ",
        TextSubstitution(text="ros2_control_hardware_type:=gazebo"),
        " ",
        TextSubstitution(text="attachment:="),
        attachment,
        " ",
        TextSubstitution(text="ros2_control_config_path:="),
        controller_config_path,
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
        output="screen",
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": os.path.join(rover_gazebo_path, "worlds/rubicon_model.sdf")
            + " -r -v 1 --gui-config "
            + os.path.join(rover_gazebo_path, "config/gazebo_gui.config"),
            "on_exit_shutdown": "True",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover",
            "-allow_renaming",
            "-x", "-8.8",
            "-y", "-1.96",
            "-z", "4.5",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "300",
            "--switch-timeout", "60",
            "--controller-manager", "/controller_manager",
        ],
    )

    drive_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drive_controller",
            "--switch-timeout", "60",
            "-c", "/controller_manager",
        ],
        output="screen",
    )

    rover_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover2_arm"),
                "launch",
                "rover_arm.launch.py",
            ])
        ),
        launch_arguments={
            "hardware_type": "gazebo",
            "launch_ros2_control": "False",
            "use_sim_time": use_sim_time,
            "attachment": attachment,
        }.items(),
        condition=IfCondition(PythonExpression(["'", attachment, "' == 'arm'"])),
    )

    gazebo_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": PathJoinSubstitution([
                FindPackageShare("rover_gazebo"),
                "config",
                "ros2_gz_bridge.yaml",
            ]),
        }],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy_to_drive = Node(
        package="rover2_control",
        executable="joy_to_drive",
        name="joy_to_drive",
        output="screen",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(rover_gazebo_path, "config/rviz_config.rviz")],
    )

    d405_stream_node = Node(
        package="rover2_camera",
        namespace="rover2_camera",
        executable="camera_ros2_conversion",
        name="d405_sim_conversion",
        parameters=[{
            "image_topic": "/camera/d405/color/image_raw",
            "cap_width": 640,
            "cap_height": 480,
            "cap_framerate": 30,
            "preset_level": 1,
            "bitrate": 4000000,
            "stream_width": 640,
            "stream_height": 480,
            "fec_percentage": 30,
            "udp_host": "127.0.0.1",
            "udp_port": 42074,
            "mux_port": 20000,
            "encoder_type": "software",
        }],
        output="screen",
        respawn=True,
        condition=IfCondition(PythonExpression(["'", attachment, "' == 'arm'"])),
    )

    d455_stream_node = Node(
        package="rover2_camera",
        namespace="rover2_camera",
        executable="camera_ros2_conversion",
        name="d455_sim_conversion",
        parameters=[{
            "image_topic": "/camera/d455/color/image_raw",
            "cap_width": 640,
            "cap_height": 480,
            "cap_framerate": 30,
            "preset_level": 1,
            "bitrate": 4000000,
            "stream_width": 640,
            "stream_height": 480,
            "fec_percentage": 30,
            "udp_host": "127.0.0.1",
            "udp_port": 42075,
            "mux_port": 20001,
            "encoder_type": "software",
        }],
        output="screen",
        respawn=True,
    )

    return LaunchDescription([
        use_sim_time_arg,
        attachment_arg,
        gazebo_node,
        robot_state_publisher_node,
        gazebo_bridge_node,
        spawn_robot,
        joy_node,
        joy_to_drive,
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=drive_controller_node,
                on_exit=[
                    rover_arm_launch,
                    gps_node,
                    imu_node,
                    rviz_node,
                    d405_stream_node,
                    d455_stream_node,
                ],
            )
        ),
    ])
