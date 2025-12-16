#!/usr/bin/env python3


# Latency measure service in ROS 2
#
# latency.py
#
# Wyatt Boer



# Pull in the stuff we need from rclpy.
import rclpy
from rclpy.node import Node

# Service call type
from rover2_status_interface.srv import LatencySize


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
class Service(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('latency_size')

		# Create a service, with a type, name, and callback.
		self.service = self.create_service(LatencySize, 'latency_size', self.callback)

	# This callback will be called every time that the service is called.
	def callback(self, request, response):
    
		response.receivetime = self.get_clock().now()

		return response


# This is the entry point for the node.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# The ROS2 idiom is to encapsulate everything in a class derived from Node.
	service = Service()

	# Spin with the node, and explicily call shutdown() when we're done.
	rclpy.spin(service)
	rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == '__main__':
	main()
