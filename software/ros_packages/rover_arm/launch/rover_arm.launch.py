import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder



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


    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="main",
        description="Ros2 Control Hardware Interface Type [main, sim]",
    )

    kinematics_yaml = load_yaml(
        "rover_arm", "config/kinematics.yaml"   
    )

    #ros2_control_hardware_type = LaunchConfiguration(ros2_control_hardware_type)
    moveit_config = (
        MoveItConfigsBuilder("rover_arm", package_name="rover_arm")
        .robot_description(
            file_path="config/rover_arm.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/rover_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group", 
        executable="move_group",
        output="screen", 
        parameters=[
            moveit_config.to_dict(),
            {
                "octomap_frame": "base_link",
                "octomap_resolution" : 0.005,
                "max_range" : 0.5,
            },    
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
        # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover_arm"),
        "config",
        "ros2_controllers.yaml",
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

    rover_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_arm_controller", "-c", "/controller_manager"],
    )

    rover_gripper_controller_spawner = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["rover_gripper_controller", "-c", "/controller_manager"],
    )

    #RViz
    rviz_config_file = (
        get_package_share_directory("rover_arm") + "/config/rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("rover_arm", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    sensor_yaml = load_yaml("rover_arm", "config/sensors_3d.yaml")

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.to_dict(),
            sensor_yaml,
        ],
        output="screen",
    )

    joy_to_servo_node = Node(
        package="joy_to_servo",
        executable="joy_to_servo_node",
    )

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{
            'pointcloud.enable': True,
            'align_depth.enable': True,
        }],
        output='screen'
    )


    return LaunchDescription(
        [
            ros2_control_hardware_type, 
            rviz_node,
            container,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            rover_arm_controller_spawner,
            joy_to_servo_node,
            servo_node,
            camera_node,
            

        ]
    )