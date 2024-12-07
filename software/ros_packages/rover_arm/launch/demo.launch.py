import os
import yaml
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    servo_yaml = load_yaml("rover_arm", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}    
    moveit_config = MoveItConfigsBuilder("rover_arm", package_name="rover_arm").to_moveit_configs()
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover_arm"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        
    )
    joy_to_servo_node = Node(
        package="joy_to_servo",
        executable="joy_to_servo_node",
    )
    ld = generate_demo_launch(moveit_config)
    ld.add_action(ros2_controllers_node)
    ld.add_action(servo_node)
    ld.add_action(joy_to_servo_node)
    ld.add_action(joy_node)
    return ld
