#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names

# We're going to publish a NodesTopics custom message
from rover2_status_interface.msg import NodesTopics

class NodeTopicDetector(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('node_topic_detector')

		# Create a publisher, and assign it to a member variable.T
		# The call takes a type, topic name, and queue size.
		self.pub = self.create_publisher(NodesTopics, 'nodetopiclisten', 10)

		#Create a timer for the publisher.
		self.publishing_period = 1 
		self.timer = self.create_timer(self.publishing_period, self.callback)
		
		#Maintain these data structures for nodes/topics seen, so we can register what's alive/dead:
		self.seen_topic_list = []
		self.seen_node_list = []

	# This callback will be called every time the timer fires.
	def callback(self):
		
		msg = NodesTopics()

		#checks what current topic list is
		self.current_topic_list = []
		topic_list = self.get_topic_names_and_types()
		for info in topic_list: #May want to filter this somehow?
			self.current_topic_list.append(info[0])
		
		#updates list of previously seen topics
		for topic in self.current_topic_list:			
			if topic in self.seen_topic_list:
				pass
			else:
				self.seen_topic_list.append(topic)

		#creates list of topic status for output - 2nd list instead of dictionary because of ROS msg typing
		self.topic_status = []
		#compares current topics with all previously seen topics
		for topic in self.seen_topic_list:
			if topic in self.current_topic_list: #true if previously seen topic is currently seen
				self.topic_status.append('Active')
			else:
				self.topic_status.append('Inactive')

		#checks what current node list is
		self.current_node_list = []
		node_list = self.get_node_names()
		for info in node_list:
			if not info[0].startswith('_'): #Filters out ros daemon/cli, doesn't filter out rqt
				self.current_node_list.append(info)
		
		#updates list of previously seen nodes
		for node in self.current_node_list:
			if node in self.seen_node_list:
				pass
			else:
				self.seen_node_list.append(node)

		#creates list of node status for output - 2nd list instead of dictionary because of ROS msg typing
		self.node_status = []
		#compares current nodes with all previously seen nodes
		for node in self.seen_node_list:
			if node in self.current_node_list: #true if previously seen node is currently seen
				self.node_status.append('Alive')
			else:
				self.node_status.append('Dead')

		# Fill in custom message fields
		msg.topic_name = self.seen_topic_list
		msg.topic_status = self.topic_status
		msg.node_name = self.seen_node_list
		msg.node_status = self.node_status
				
		self.pub.publish(msg)		

		self.get_logger().info(f'published: {msg}')

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Make a node class.
	detect = NodeTopicDetector()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(detect)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
