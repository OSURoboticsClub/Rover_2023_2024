from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

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
    urdf_path = PathJoinSubstitution([FindPackageShare(pkg_name), 'rover2_control', 'drive_control.xacro'])
    controller_config = PathJoinSubstitution([FindPackageShare(pkg_name), 'rover2_control', 'odrive_drive_ros2_control.yaml'])

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        remappings=[('/joy', 'joy_can_drive')]
    )

    # Joy Node to convert joystick input to velocities
    joy_to_drive_node = Node(
        package='rover2_control',  # Replace with your package name
        executable='joy_to_drive',  # This is the node you created above
        name='joy_to_drive',
        output='screen'
    )

    # Load joint_state_broadcaster after ros2_control_node is up
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    can_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['can_controller', "-c", "/controller_manager"],
        output='screen'
    )

    auton_controller = Node(
        package='rover2_control',
        executable="auton_controller",
        name="auton_controller",
        **config
    )
    gripper_can_control_node = Node(
        package='rover2_control',
        executable='gripper_control',
        name='gripper_can_control',
        parameters=[{
            'is_position_control': False,
            'joy_publish_rate': 50,
            'can': 'can1'
        }],
        **config
    )

    return LaunchDescription([
        # Robot State Publisher
        
        
        # Joy Node to convert joystick input to velocities

        Node(
            package='rover2_control',  # Replace with your package name
            executable='drive_can_control',  # This is the node you created above
            name='drive_can_control',
            **config
        ),
        # IRIS controller
        Node(
            package='rover2_control',
            executable='iris_controller',
            name='iris_controller',
            parameters=[{
                '~port': '/dev/rover/ttyIRIS',
                '~hertz': 20
            }],
            **config
        ),
        # Load joint_state_broadcaster after ros2_control_node is up
  
        Node(
            package='rover2_control',
            executable='chassis_pan_tilt_control',
            name='chassis_pan_tilt',
            **config
        ),
        Node(
            package='rover2_control',
            executable='monitor_aruco',
            name='monitor_aruco',
            **config
        ),
       # Node(
       #     package='rover2_control',
       #     executable='drill_control',
       #     name='drill_control',
       #     **config
       # ),
        Node(
            package='rover2_control',
            executable='tower_pan_tilt_control',
            name='tower_pan_tilt',
            **config
        ),
       # Node(
       #     package='rover2_control',
       #     executable='effectors_control',
       #     name='effectors',
       #     **config
       # ),
        Node(
            package='rover2_control',
            executable='joint_position_control',
            name='joint_position',
            **config
        ),
        #Node(
        #    package='rover2_control',
        #    executable='drive_control',
        #    name='rear_bogie',
        #    parameters=[{
        #        '~port': '/dev/rover/ttyEffectors',
        #        '~scimech_control_topic_main_actuator': 'scimech_control/main_actuator',
        #        '~scimech_control_topic_flexinol': 'scimech_control/flexinol',
        #        '~scimech_control_topic_secondary_actuator': 'scimech_control/secondary_actuator',
        #        '~drive_control_status_topic': 'drive_status/rear',
        #        '~first_motor_id': 1,
        #        '~second_motor_id':2,
        #        '~third_motor_id':3,
        #    }],
        #    **config
        #),
        gripper_can_control_node,
        auton_controller
    ])
