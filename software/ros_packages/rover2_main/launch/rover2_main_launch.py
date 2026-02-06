import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
   drive_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_control')),
         '/odrive_drive_control_launch.py'])
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
         get_package_share_directory('rover_arm'),
         'launch'), '/rover_arm.launch.py'])
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
   nav_autonomy = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav_autonomy'),
         'launch'), '/nav_launch.py'])
      )
   return LaunchDescription([
      drive_control,
      imu,
      arm,
      status,
      mapping,
      nav_autonomy,
      cameras,

   ])
