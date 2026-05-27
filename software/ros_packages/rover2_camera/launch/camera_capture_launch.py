import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

DRIVE_IP = '192.168.1.6'
ARM_IP = '192.168.1.8'

def generate_launch_description():
    # Replace these with your camera serial numbers
    # You can find them with `rs-enumerate-devices`


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
        "pointcloud__neon_.enable": True,
        "pointcloud__neon_.stream_filter": 2,
        "align_depth.enable": True,
        "depth_fps": 10,
        "rgb_fps": 10,
	}],
        output='screen'
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
            "pointcloud__neon_.enable": True,
            "pointcloud__neon_.stream_filter": 2,
            "align_depth.enable": True,
            #"enable_rgbd": True,
            "decimation_filter": True,
            "decimation_filter.filter_magnitude": 8,
            "enable_sync": True,
            "pointcloud.stream_filter": 2,
            "enable_color": True,
            # "enable_depth": True,
            "serial_no":"218622273613",
            "depth_fps": 5,
            "rgb_fps": 5,
        }],
        output='screen'
    )

    chassis_left_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='chassis_left_cam',
        parameters=[{
            'device': '/dev/rover/camera_left_chassis',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 25,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': DRIVE_IP,
            'udp_port': 42069,
            'mux_port': 20002
        }],
        respawn=True
    )
    chassis_right_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='chassis_right_cam',
        parameters=[{
            'device': '/dev/rover/camera_right_chassis',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 25,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 100,
            'udp_host': DRIVE_IP,
            'udp_port': 42070,
            'mux_port': 20003
        }],
        respawn=True
    )

    tower_gimbal_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='tower_gimbal_cam',
        parameters=[{
            'device': '/dev/rover/camera_tower_gimbal',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 100,
            'udp_host': ARM_IP,
            'udp_port': 42068,
            'mux_port': 20000
        }],
        respawn=True
    )

    back_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='back_cam',
        parameters=[{
            'device': '/dev/rover/camera_rear',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 25,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 100,
            'udp_host': DRIVE_IP,
            'udp_port': 42071,
            'mux_port': 20000
        }],
        respawn=True
    )
    birds_eye_cam_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='birds_eye_cam',
        parameters=[{
            'device': '/dev/rover/camera_bird_eye',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 25,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 100,
            'udp_host': ARM_IP,
            'udp_port': 42072,
            'mux_port': 20000
        }],
        respawn=True
    )

    #pan_tilt_ir
    pan_tilt_cam_node = Node(
       package='rover2_camera',
       namespace='rover2_camera',
       executable='camera_capture',
       name='pan_tilt_cam',
       parameters=[{
           'device': '/dev/rover/camera_pan_tilt',
           'cap_width': 640,
           'cap_height': 480,
           'cap_framerate': 30,
           'preset_level': 1,
           'bitrate': 4000000,
           'stream_width': 640,
           'stream_height': 480,
           'fec_percentage': 30,
           'udp_host': DRIVE_IP,
           'udp_port': 42073,
           'mux_port': 20000
       }],
       respawn=True
    )

    realsense_gstream = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_ros2_conversion',
        name='realsense_conversion',
        parameters=[{
            'image_topic': '/camera/d405/color/image_raw',
            'cap_width': 640,
            'cap_height': 480,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': ARM_IP,
            'udp_port': 42074,
            'mux_port': 20000
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
        # realsense_launch_nav,
#        ir_camera_node,
#        ir_camera_node,
#        main_nav_node,
        chassis_right_cam_node,
        chassis_left_cam_node,
        birds_eye_cam_node,
        tower_gimbal_cam_node,
        back_cam_node,
        pan_tilt_cam_node,
        muxing_node,
        d405_node,
        realsense_gstream,
    ])

