import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Replace these with your camera serial numbers
    # You can find them with `rs-enumerate-devices`


#    realsense_launch_nav = Node(
#        package='realsense2_camera',
#        executable='realsense2_camera_node',
#        name='d455',
#        parameters=[{
#            "camera_name": "d455",
#            
#            "pointcloud.enable": True,
#            "align_depth.enable": True,
#            "serial_no":"318122302525",
#        }],
#        output='screen'
#    )
    realsense_launch_nav = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455',
	parameters=[{
	    "camera_name": "d455",
	    "serial_no": "318122302525",
	    "depth_module.depth_profile": "424x240x5",  
	    "depth_module.infra_profile": "424x240x5", 
	    "rgb_camera.color_profile": "424x240x5",
	    "pointcloud.enable": True,
	    "align_depth.enable": True,
	}],
        output='screen'
    ) 
   # Your rover2_camera nodes
    ir_camera_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='ir',
        parameters=[{
            'device': '/dev/rover/camera_infrared',
            'cap_width': 1920,
            'cap_height': 1080,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42067
        }],
        respawn=True
    )
    gripper_rgb_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='gripper_rgb',
        parameters=[{
            'device': '/dev/rover/gripper-rgb',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42067
        }],
        respawn=True
    )

    main_nav_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='main_navigation',
        parameters=[{
            'device': '/dev/rover/camera_main_navigation',
            'cap_width': 1920,
            'cap_height': 1080,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42068
        }],
        respawn=True
    )

    return LaunchDescription([
        realsense_launch_nav,
#        ir_camera_node,
#        main_nav_node,
#        gripper_rgb_node
    ])

