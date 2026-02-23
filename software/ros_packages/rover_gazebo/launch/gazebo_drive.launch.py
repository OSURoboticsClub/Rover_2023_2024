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

    world_path = os.path.join(rover_gazebo_path, 'worlds/rubicon.sdf')   

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

        tf2_node,
        joy_node,
        gazebo_bridge_node,
        spawn_robot,
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
        # drive_controller_node,
        joy_to_drive,
        robot_state_publisher_node,
    ])
