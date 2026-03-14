import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    drive_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_control')),
         '/odrive_ros2_control.launch.py']
      ),
      launch_arguments={
          "launch_ros2_control":"False"
      }.items()
   )

    cameras = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_camera'),
         'launch'), '/camera_capture_launch.py'])
      )

    imu = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_odometry')),
         '/rover2_odometry_launch.py'])
      )

    arm = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_arm'),
         'launch'), '/rover_arm.launch.py']
      ),
      launch_arguments={
          "launch_ros2_control":"False"
      }.items()
   )

    status = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_status'),
         'launch'), '/rover2_status_launch.py'])
      )

    mapping = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav_autonomy'),
         'launch'), '/mapping_launch.py'])
      )
   # Timer action to delay the listener node
    delay_mapping = TimerAction(
         period=5.0,  # Delay in seconds
         actions=[mapping]
      )
    nav_autonomy = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav_autonomy'),
         'launch'), '/nav_launch.py'])
      )
    state_publisher = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav_autonomy'),
         'launch'), '/state_publisher_launch.py'])
      )






    robot_description = Command([
        FindExecutable(name="xacro"), 
        ' ',
        PathJoinSubstitution([
            FindPackageShare('rover2_arm'),
            'config',
            'rover.urdf.xacro'
        ]),
        ' ',
        TextSubstitution(text='ros2_control_hardware_type:=main'),
        ' ',
        TextSubstitution(text='attachment:=arm'),
    ])



    ros2_controllers_path = os.path.join(
        get_package_share_directory("rover2_main"),
        "config",
        "rover_control.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        **config
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'use_sim_time': "false",
            "robot_description": robot_description
        }],
        output="screen",
    )
    
    # Load joint_state_broadcaster after ros2_control_node is up
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


    return LaunchDescription([
#      state_publisher,
      ros2_control_node,
      robot_state_publisher_node,
      joint_state_broadcaster_spawner,
      drive_control,
      imu,
      arm,
      status,
#      cameras,
      nav_autonomy,
      delay_mapping,
   ])
