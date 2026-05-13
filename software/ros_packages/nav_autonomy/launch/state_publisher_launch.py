from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare("rover_arm"),
        "config",
        "rover_arm.urdf.xacro",
    ])

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
