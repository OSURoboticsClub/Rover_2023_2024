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
from launch.conditions import IfCondition

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
        "hardware_type",
        default_value="main",
        description="Ros2 Control Hardware Interface Type [main, sim]",
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description="Use simulation time"
    )

    controller_type= DeclareLaunchArgument(
        "controller_type",
        default_value="xbox",
        description="Ros2 Control Hardware Interface Type [xbox, ps, flight]",
    )

    launch_ros2_control = LaunchConfiguration('launch_ros2_control', default='true')
    launch_ros2_control_arg = DeclareLaunchArgument(
            'launch_ros2_control',
            default_value='true',
            description='Launch arm with indendent ros2 control[default:true]'
    )

    attachment = LaunchConfiguration('attachment', default='arm')
    attachment_arg = DeclareLaunchArgument(
        'attachment',
        default_value='arm',
        description="Attachment type"
    )

    kinematics_yaml = load_yaml(
        "rover2_arm", "config/kinematics.yaml"   
    )

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    octomap_config = {
        'octomap_frame': 'rover_arm_base_link',
        'octomap_resolution': 0.05,
        'max_range': 5.0
    }

    # octomap_sensor_config = load_yaml('rover2_arm', 'config/sensors_3d.yaml')
    
    #ros2_control_hardware_type = LaunchConfiguration(ros2_control_hardware_type)
    moveit_config = (
        MoveItConfigsBuilder("rover2_arm", package_name="rover2_arm")
        .robot_description(
            file_path="config/rover.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "hardware_type"
                ),
                "attachment": LaunchConfiguration("attachment"),
            },
        )
        .robot_description_semantic(file_path="config/rover.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
            load_all=False
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # chomp_configs = load_yaml(
    #     "rover2_arm", "config/chomp_interface_planning.yaml"
    # )

    # ompl_configs = load_yaml(
    #     "rover2_arm", "config/ompl_planning.yaml"
    # )
    # pilz_configs = load_yaml(
    #     "rover2_arm", "config/pilz_planning.yaml"
    # )
    
    # octomap_updater_config = load_yaml('rover2_arm', 'config/sensors_3d.yaml')

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True}

    move_group_node = Node(
        package="moveit_ros_move_group", 
        executable="move_group",
        output="screen", 
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
            octomap_config,
            # octomap_sensor_config,            
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
        # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover2_arm"),
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
        condition=IfCondition(launch_ros2_control)
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
        condition=IfCondition(launch_ros2_control)
    )

    moveit_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_arm_controller_moveit", 
                   "-c", "/controller_manager",
                   "--inactive",
        ],
    )

    rover_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_arm_controller", 
                   "-c", "/controller_manager",
        ],
    )

    #RViz
    rviz_config_file = (
        get_package_share_directory("rover2_arm") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("rover2_arm", "config/servo_parameters.yaml")
    # Get parameters for the Servo node
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    # octomap_config,
                    # octomap_updater_config,
                    kinematics_yaml,
                ],
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
            # ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/rover_base_origin", "frame_id": "/map"}],
            # ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            ComposableNode(
                package="rover_arm_control_interface",
                plugin="moveit_servo::JoyToServoPub",
                name="joy_to_servo_pub_node",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {'controller_type': LaunchConfiguration('controller_type')},
                ],
            ),
        ],
        output="screen",
    )

    controller_switcher_node = Node(
        package="joy_to_servo",
        executable="controller_switcher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    d405_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d405',
        parameters=[{
            "camera_name": "d405",
            "depth_width": 1280,
            "depth_height": 720,
            "color_width": 1280,
            "color_height": 720,
            "pointcloud.enable": True,
            "align_depth.enable": True,
            #"enable_rgbd": True,
            "decimation_filter": True,
            "decimation_filter.filter_magnitude": 8,
            "enable_sync": True,
            "pointcloud.stream_filter": 2,
            # "enable_color": True,
            # "enable_depth": True,
            # "serial_no":"_218622273613",
            "depth_fps": 5,
            "rgb_fps": 5,
        }],
        output='screen'
    )

    d455_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455',
        parameters=[{
            "camera_name": "d455",
            "depth_width": 1280,
            "depth_height": 720,
            "color_width": 1280,
            "color_height": 720,
            "pointcloud.enable": True,
            "align_depth.enable": True,
            "serial_no":"_318122302525",
            "depth_fps": 10,
            "rgb_fps": 10,
        }],
        output='screen'
    )

    joy_to_servo_node = Node(
        package="joy_to_servo",
        executable="joy_to_servo_node",
        parameters=[{
            'controller_type': LaunchConfiguration('controller_type')
        }, {
            'use_sim_time': use_sim_time,
        }]
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'use_sim_time': use_sim_time,},
            moveit_config.robot_description,
            ],
        output="screen",
        condition=IfCondition(launch_ros2_control)
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            launch_ros2_control_arg,
            ros2_control_hardware_type, 
            attachment_arg, 
            controller_type,
            move_group_node,
            ros2_control_node,
            robot_state_publisher_node,
            container,
            joint_state_broadcaster_spawner,
            rover_arm_controller_spawner,
            moveit_arm_controller_spawner,
            controller_switcher_node,
            rviz_node,
            #joy_to_servo_node,
            # d405_node,
            #d455_node,
        ]
    )
