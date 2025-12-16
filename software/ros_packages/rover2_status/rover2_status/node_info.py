#!/usr/bin/env python3


# service that returns info about a selected node
#
# node_info.py
#
# Wyatt Boer



import rclpy
from rclpy.node import Node

# Service call type
from rover2_status_interface.srv import NodeInfo

class NodeInfoService(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('node_info')

		# Create a service, with a type, name, and callback.
		self.service = self.create_service(NodeInfo, 'node_info', self.callback)

		#List of default topics to ignore:
		self.pub_ignore = ["/parameter_events","/rosout"]
		self.sub_ignore = ["/parameter_events"]
		#List of default service types to ignore:
		self.service_ignore = ["rcl_interfaces/srv/DescribeParameters",
							   "rcl_interfaces/srv/GetParameterTypes",
							   "rcl_interfaces/srv/GetParameters",
							   "rcl_interfaces/srv/ListParameters",
							   "rcl_interfaces/srv/SetParameters",
							   "rcl_interfaces/srv/SetParametersAtomically",
							   "type_description_interfaces/srv/GetTypeDescription"]

	# This callback will be called every time that the service is called.
	def callback(self, request, response):
		
		#Requested nodename and namespace:
		self.nodename = request.node
		self.namespace = '/' #request.nodenamespace

		# fills in response data with respective functions
		response.pub_topics = self.get_pub_topics()
		response.sub_topics = self.get_sub_topics()
		response.services = self.get_services()

		self.get_logger().info(f'responded with: {response}')

		return response

	# returns a list of the topics being published by selected node
	def get_pub_topics(self):
		output_list = []
		pub_topic_list = self.get_publisher_names_and_types_by_node(self.nodename,self.namespace)
		for topic in pub_topic_list:
			if topic[0] not in self.pub_ignore:
				output_list.append(topic[0])
		return output_list
	# returns a list of the topics being subscribed to by selected node
	def get_sub_topics(self):
		output_list = []
		sub_topic_list = self.get_subscriber_names_and_types_by_node(self.nodename,self.namespace)
		for topic in sub_topic_list:
			if topic[0] not in self.sub_ignore:
				output_list.append(topic[0])
		return output_list
	# returns a list of the service server topics for selected node
	def get_services(self):
		output_list = []
		sub_service_list = self.get_service_names_and_types_by_node(self.nodename,self.namespace)
		for service in sub_service_list:
			self.get_logger().info(f"service type: {service[1]}")
			if service[1][0] not in self.service_ignore:
				output_list.append(service[0])
		return output_list

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Make a node class.
	info = NodeInfoService()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(info)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
