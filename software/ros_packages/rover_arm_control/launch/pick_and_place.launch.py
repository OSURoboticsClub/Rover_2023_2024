from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

import os


def generate_launch_description():
    ros2_control_hardware_type = DeclareLaunchArgument(
        "hardware_type",
        default_value="main",
        description="Ros2 Control Hardware Interface Type [main, sim]",
    )

    controller_type= DeclareLaunchArgument(
        "controller_type",
        default_value="xbox",
        description="Ros2 Control Hardware Interface Type [xbox, ps, flight]",
    )

    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }
    arm = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover_arm'),
         'launch'), '/rover_arm.launch.py']),
        launch_arguments={
            'hardware_type': LaunchConfiguration(
                    "hardware_type",
            ),
            'controller_type': LaunchConfiguration(
                    "controller_type",
            )
        }.items()

    )
    
    gripper_can_control_node = Node(
        package='rover_arm_control',
        executable='gripper_control',
        name='arm_gripper_control',
        parameters=[{
            'is_position_control': False,
            'joy_publish_rate': 50,
            'can': "can0"
        }],
        **config
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output='screen',
        respawn= True
    )
    relative_move_node = Node(
        package='rover_arm_control',
        executable='relative_move',
        name='relative_move',
        **config
    )
    absolute_move_node = Node(
        package='rover_arm_control',
        executable='absolute_move',
        name='absolute_move',
        **config
    )
    pick_and_place_node = Node(
        package='rover_arm_control',
        executable='pick_and_place',
        name='pick_and_place',
        **config
    )

    aruco_detector_node = Node(
        package='rover_arm_control',
        executable='aruco_detector',
        name='aruco_detector',
        **config
    )

    #Launch the pc_filter node
    pc_filter_node = Node(
			package = 'pc_processing',
			executable = 'pc_filter',
			name = 'pc_filter',
			remappings = [
				#('/raw_point_cloud', '/astra_ros/devices/default/point_cloud') #Remap for old HW rosbag
				('/raw_point_cloud','/camera/d405/depth/color/points') #Remap for rover d405 pointcloud
			]
    )
    #Launch the plane fitting node
    pc_plane_node = Node(
			package = 'pc_processing',
			executable = 'plane_fit',
			name = 'plane_fit',
    )
    #Launch the object clustering node
    pc_count_object_node = Node(
			package = 'pc_processing',
			executable = 'count_objects',
			name = 'count_objects'
    )



    return LaunchDescription([
        ros2_control_hardware_type,
        controller_type,
        arm,
        gripper_can_control_node,
        joy_node,
        relative_move_node,
        absolute_move_node,
        #pick_and_place_node,
        aruco_detector_node,
        # pc_filter_node,
        # pc_plane_node,
        # pc_count_object_node,

    ])