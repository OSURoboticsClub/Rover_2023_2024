import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Replace these with your camera serial numbers
    # You can find them with `rs-enumerate-devices`
    pkg_share = get_package_share_directory('rover2_camera')
    left_calib = os.path.join(pkg_share, 'calibration', 'camera_left_chassis', 'calibration_data.yaml')
    right_calib = os.path.join(pkg_share, 'calibration', 'camera_right_chassis', 'calibration_data.yaml')
    
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
            # "depth_module.depth_profile": "424x240x5",  
            # "depth_module.infra_profile": "424x240x5", 
            # "rgb_camera.color_profile": "424x240x5",
            
            # Test for better costmap clearing
            #"depth_module.profile": "848x480x15",
            #"rgb_camera.profile": "848x480x15",
            # "depth_module.emitter_enabled": True, 
            # "depth_module.laser_power": 360,
            # "depth_module.enable_auto_exposure": True,
            # "pointcloud.enable": False,

            "depth_width": 1280,
            "depth_height": 720,
            "color_width": 1280,
            "color_height": 720,
            "pointcloud.enable": True,
            "align_depth.enable": True,
            "depth_fps": 10,
            "rgb_fps": 10,
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
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42067,
            'mux_port': 20001
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
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.104',
            'udp_port': 42068,
            'mux_port': 20000
        }],
        respawn=True
    )
    chassis_left_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='chassis_left_cam',
        parameters=[left_calib,{
            'device': '/dev/rover/camera_left_chassis',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 25,
            'preset_level': 1,
            'bitrate': 4000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42069,
            'mux_port': 20002,
        }],
        respawn=True
    )
    chassis_right_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='chassis_right_cam',
        parameters=[right_calib,{
            'device': '/dev/rover/camera_right_chassis',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 25,
            'preset_level': 1,
            'bitrate': 4000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 100,
            'udp_host': '192.168.1.1',
            'udp_port': 42070,
            'mux_port': 20003,
            
        }],
        respawn=True
    )

    muxing_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_muxing',
        name='muxing_node',
        respawn=True
    )

    return LaunchDescription([
        realsense_launch_nav,
#        ir_camera_node,
#        main_nav_node,
#        gripper_rgb_node,
        chassis_right_cam_node,
        chassis_left_cam_node,
        muxing_node
        # muxing_node
    ])

