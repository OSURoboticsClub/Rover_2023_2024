from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="main",
        description="Ros2 Control Hardware Interface Type [main, sim]",
    )
#    use_sim_time = DeclareLaunchArgument(
#        "use_sim_time",
#        default_value="true",
#        description="Use simulated time for all nodes",
#    )
    
    moveit_config = (MoveItConfigsBuilder("rover_arm", package_name="rover_arm")
        .robot_description(
            file_path="config/rover_arm.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .to_moveit_configs()
    )

#    d405_node = Node(
#        package="realsense2_camera",
#        executable="realsense2_camera_node",
#        name="camera",
#        output="screen",
#        parameters=[{
#            "use_sim_time": LaunchConfiguration("use_sim_time"),
#            "camera_name": "camera",
#            "enable_depth": True,
#            "enable_color": True,
#            "enable_infra1": False,
#            "enbale_infra2": False,
#           "depth_width": 848,
#            "depth_height": 480,
#            "color_width": 848,
#            "color_height": 480,
#            "pointcloud.enable": True,
#            "align_depth.enable": True,
#            "camera.depth.enabled": True,  # Ensure depth camera is enabled
#            "camera.color.enabled": True,  # Ensure color camera is enabled
#            "camera.aligned_depth_to_color.enabled": True,  # Enable aligned depth to color
#            "camera.extrinsics.depth_to_color.enabled": True  # Enable extrinsics for depth to color
#        }]
#    )

    launch_description = generate_demo_launch(moveit_config)
    #launch_description.add_action(use_sim_time)
    #launch_description.add_action(d405_node)

    launch_description.add_action(ros2_control_hardware_type)
    
    return launch_description
