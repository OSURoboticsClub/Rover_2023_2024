from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    pkg_name = 'rover2_control'  # <- update this to your package name
    controller_config = PathJoinSubstitution([FindPackageShare(pkg_name), 'rover2_control', 'odrive_drive_ros2_control.yaml'])
    rover_model_path = os.path.join(get_package_share_directory('rover_urdf'), '..')

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
    )
    hardware_type = LaunchConfiguration('hardware_type', default='main')
    hardware_type_arg = DeclareLaunchArgument(
            'hardware_type',
            default_value='main',
            description='Robot Hardware [main, sim, gazebo]'
    )

    launch_ros2_control = LaunchConfiguration('launch_ros2_control', default='true')
    launch_ros2_control_arg = DeclareLaunchArgument(
            'launch_ros2_control',
            default_value='true',
            description='Launch drivetrain with indendent ros2 control[default:true]'
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
        TextSubstitution(text='ros2_control_hardware_type:='),
        hardware_type,
        ' ',
        TextSubstitution(text='attachment:=arm'),
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
        condition=IfCondition(launch_ros2_control)
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
        condition=IfCondition(launch_ros2_control)
    )
    
   #drive controller
    drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller",  "-c", "/controller_manager"],
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
        condition=IfCondition(launch_ros2_control),
        **config
    )


    joy_to_drive = Node(
        package="rover2_control",
        executable="joy_to_drive",
        name="joy_to_drive",
        **config
    )

    auton_controller = Node(
        package='rover2_control',
        executable="auton_controller",
        name="auton_controller",
        **config
    )

    # gripper_can_control_node = Node(
    #     package='rover2_control',
    #     executable='gripper_control',
    #     name='gripper_can_control',
    #     parameters=[{
    #         'is_position_control': False,
    #         'joy_publish_rate': 50,
    #         'can': 'can1'
    #     }],
    #     **config
    # )

    return LaunchDescription([
        # Robot State Publisher
        
        # Joy Node to convert joystick input to velocities

        # Node(
        #     package='rover2_control',  # Replace with your package name
        #     executable='drive_can_control',  # This is the node you created above
        #     name='drive_can_control',
        #     **config
        # ),
        # IRIS controller
        use_sim_time_arg,
        hardware_type_arg,
        launch_rover2_control_arg,
        Node(
            package='rover2_control',
            executable='iris_controller',
            name='iris_controller',
            parameters=[{
                '~port': '/dev/ttyUSB0',
                '~hertz': 30
            }],
            **config
        ),
        # Load joint_state_broadcaster after ros2_control_node is up
  
        #Node(
        #    package='rover2_control',
        #    executable='chassis_pan_tilt_control',
        #    name='chassis_pan_tilt',
        #    **config
        #),
#        Node(
#            package='rover2_control',
#            executable='monitor_aruco',
#            name='monitor_aruco',
#            **config
#        ),
        # Node(
        #     package='rover2_control',
        #     executable='tower_pan_tilt_control',
        #     name='tower_pan_tilt',
        #     **config
        # ),
        #Node(
        #    package='rover2_control',
        #    executable='joint_position_control',
        #    name='joint_position',
        #    **config
        #),
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        drive_controller,
        joy_to_drive,
        # gripper_can_control_node,
#        auton_controller
    ])
