#!/usr/env/bin python3

#Launch file for testing our ROB514 pointcloud processing pipeline, opens the relevant nodes and rviz config. 

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
	
	return LaunchDescription([
		#Launch the pc_filter node
		Node(
			package = 'pc_processing',
			executable = 'pc_filter',
			name = 'pc_filter',
			remappings = [
				#('/raw_point_cloud', '/astra_ros/devices/default/point_cloud') #Remap for old HW rosbag
				('/raw_point_cloud','/camera/d405/depth/color/points') #Remap for rover d405 pointcloud
			]
		),
		#Launch the plane fitting node
		Node(
			package = 'pc_processing',
			executable = 'plane_fit',
			name = 'plane_fit',
		),
		#Launch the object clustering node
		Node(
			package = 'pc_processing',
			executable = 'count_objects',
			name = 'count_objects'
		),
		#Launch Rviz2:
		Node(
			package = 'rviz2',
			executable = 'rviz2',
			name = 'rviz2',
			#arguments = ['-d' +  os.path.join(get_package_share_directory('pc_processing'),'config/bartender.rviz')] #rviz config for old HW rosbag	
			arguments = ['-d' +  os.path.join(get_package_share_directory('pc_processing'),'config/pc_processing.rviz')] #rviz config for testing rover d405 pointcloud
        )

	])
