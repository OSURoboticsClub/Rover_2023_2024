# A node which subscribes to /rousout and filters the log messages for a specific node.
# It should return about 1 info node log per second, and any logs of greater severity:

#Written for ROB499 as part of our Rover status project:

#Import rclpy and node:
import rclpy
from rclpy.node import Node

#/rosout msg type
from rcl_interfaces.msg import Log

#paired down msg to republish:
from rover2_status_interface.msg import FilteredLog

from rover2_status_interface.srv import NodeLog

#Define the node class:
class NodeLogs(Node):
	#Initialize the class
	def __init__(self):
		#Node initialization
		super().__init__("node_logs")

		#class status variables
		self.receiving = False
		self.set_node_name = "Null"
		self.log_level_thresh = 30

		#class variables to store the last relevant Log message data:
		self.log_stamp = self.get_clock().now().to_msg()
		self.log_node_name = "Null"
		self.log_level = 0
		self.log_msg = "Null"

		#Use this to check if there is a new log to publish
		self.prev_log_stamp = self.log_stamp

		#initialize subscriber and publisher
		self.sub = self.create_subscription(Log, 'rosout', self.sub_callback, 10)

		self.pub = self.create_publisher(FilteredLog,'log_filt', 10)

		#Publisher runs on a timer, typically 1hz
		self.pub_timer = self.create_timer(1, self.pub_callback)

		#Service to start/stop logging and select a node.
		self.serv = self.create_service(NodeLog, 'node_log', self.serv_callback)
		

	#The subscriber callback will run each time we recieve a /rosout message, which is a lot of times...
	def sub_callback(self, msg):

		#Increment log stamps so we can avoid repeatedly publishing the same log
		self.prev_log_stamp = self.log_stamp
		
		#Check if we've got a new msg:
		if self.receiving and (msg.name == self.set_node_name):

			#Save the important msg data.
			self.log_stamp = msg.stamp
			self.log_node_name = msg.name
			self.log_level = msg.level
			self.log_msg = msg.msg

			#Publish Immediately if message is more important than info()
			#Otherwise Wait for next pub_callback
			if(self.log_level >= 30):
				self.pub_callback()
				
	#The publisher will run on a timer or when a higher level msg is recieved.
	def pub_callback(self):
		#Check the timestamps to avoid repeatedly publishing:
		if self.log_stamp != self.prev_log_stamp:
			
			#For debugging, it's useful to log what we're sending, however this becomes kinda circular...
			#I think the functionality of this node in particular overlaps most with rqt, 
			#however it's still worth doing for our end goal (maybe)? Worst case scenario we're building a worse rqt clone lol 
			#self.get_logger().info(f"Publishing {self.log_node_name} log with message: {self.log_msg}")
			
			#Create and fill in the message
			msg = FilteredLog()

			msg.stamp = self.log_stamp
			msg.name = self.log_node_name
			msg.level = self.log_level
			msg.msg = self.log_msg

			#publish
			self.pub.publish(msg)

	#The service callback will start/stop the node from sending logs:
	def serv_callback(self, request, response):
		#Log the request.
		self.get_logger().info(f"Got request for log filtering: {request.enable}, at {request.hz}hz, of {request.node}.")
		
		#update the receiving status, name, and frequency, etc.
		self.receiving = request.enable
		self.set_node_name = request.node
		self.pub_timer.timer_period_ns = (1/request.hz) * 1000000000

		#return an acknowledge:
		response.acknowledge = True
		return response

#Entry point for the single node:
def main(args=None):
	#Start rclpy
	rclpy.init(args=args)

	#Create class instance
	node_logs = NodeLogs()

	#Hand to rclpy:
	rclpy.spin(node_logs)

	#stop when done:
	rclpy.shutdown()
