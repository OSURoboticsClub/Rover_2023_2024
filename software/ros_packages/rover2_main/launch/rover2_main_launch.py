import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
   control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover2_control'),
         'rover2_control_launch.py')])
      )

   cameras = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rover_camera'),
         'launch'), '/camera_launch.py'])
      )

   bridge = IncludeLaunchDescription(
      XMLLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rosbridge_server'),
         'launch'), '/rosbridge_websocket_launch.xml'])
      )
   arm = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
	 get_package_share_directory('rover_arm'),
	 'launch'), '/arm.launch.py'])
      )

   return LaunchDescription([
      control,
      cameras,
      bridge,
      arm
   ])
