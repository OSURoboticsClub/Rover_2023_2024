import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

# LAUNCH: for testing without another odom source
    # ros2 launch nav_autonomy mapping_launch.py vo:=true viz:=true   

# rtabmap core node TF notes
    # Required
        # odom -> base_link
        # base_link -> ... -> <camera_name>_color_optical_frame

def generate_launch_description():
    package_name='nav_autonomy'
    pkg_share = get_package_share_directory(package_name)

    parameters = [os.path.join(pkg_share, 'config', 'rtab_params.yaml')]

    remappings=[
        ('rgb/image', '/camera/d455/color/image_raw'),
        ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw'),
        ]
        
    config_rviz = os.path.join(
        get_package_share_directory('nav_autonomy'), 'config', 'map_display_cfg.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('viz',        default_value='false',  description='Launch RTAB-Map UI and RVIZ.'),
        DeclareLaunchArgument('vo',       default_value='false', description='Visual Odometry Node for testing'),
        DeclareLaunchArgument('rviz_cfg',   default_value=config_rviz,  description='Configuration path of rviz2.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch visual odom for testing
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=IfCondition(LaunchConfiguration("vo")),
            parameters=parameters,
            remappings=remappings),

 
        # Core SLAM node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("viz")),
            parameters=parameters,
            remappings=remappings),
            
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("viz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
            # RViz Subscribed Topics
                #   /map
                #   /map_updates
                #   /mapData
                #   /mapGraph
                #   /odom_last_frame
                #   /odom_local_map
                #   /initialpose
                #   /goal_pose
                #   /clicked_point
    ])
