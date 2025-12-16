#Slip direction node for rob499 final project.

#Import rclpy and node:

import rclpy
from rclpy.node import Node

from rover2_status_interface.msg import ODriveStatus
from rover2_status_interface.msg import TractionStatus

from math import pi

#Drivetrain/ODrive ID mappings:
#(The rover has 6 direct driven wheels, these map the wheels (eg. Left Front) to the node ids)
drive_mappings = {
'LM' : 0,
'RF' : 1,
'LF' : 2,
'RM' : 3,
'LB' : 4,
'RB' : 5,
}

#Node class definition:
class SlipDetector(Node):
	def __init__(self):
		super().__init__("slip_detector")
		
		#Subscribe to a topic which does slip detection:
		self.sub = self.create_subscription(ODriveStatus, 'odrive_telem', self.sub_callback, 10)
	
		#publish our results to a topic:
		self.pub = self.create_publisher(TractionStatus, 'slip_status', 10)

		#publish messages slower than we do slip detection:
		self.pub_timer = self.create_timer(0.2, self.pub_msg)

		#Conversion from Odrive reported velocity (RPS at the motor) to m/s at the wheel:
		wheel_dia = 0.3 #meters
		gear_ratio = 50
		self.conversion = wheel_dia*pi/gear_ratio

		#A threshold value for the wheel slip ratio, should be between 0 and 1:
		self.slip_ratio_thresh = 0.2 #unitless
		self.slip_bias = 0.25 #m/s

		#We will compare with the moving average of each drive side's velocity using a ring buffer.
		self.ring_len = 3
		self.ring_idx = 0

		self.left_velo_buffer = [0] * self.ring_len
		self.right_velo_buffer = [0] * self.ring_len

		#Define a msg object to send:
		self.slip_msg = TractionStatus()

	def sub_callback(self, msg):
		#First, unpack the data we care about from the msg:

		nodes = msg.id
		velocities = [self.conversion * velo for velo in msg.velocity] #convert to m/s
		currents = msg.iq_measured

		#and create a bool array to save the slip status to:
		slip_status = [False] * len(nodes)
		names = [''] * len(nodes)

		#Second, check that we have the right number of node ids, just in case:
		if len(nodes) != 6:
			#Warn and finish the callback w/o publishing:
			self.get_logger().warn(f'Incorrect number of node ids: {node_ids} found.')
			return

		#Then, calculate the slip ratio.

		#We know that the rover is front-heavy but doesn't tip on flat ground, so the middle wheels
		#are the least likely to slip. We could treat them as reference or compare with a moving average.
		#Perhaps add some weighting in the future?
		
		#First, We're going to get the average velocity of each drive side:
		l_velos = [ velocities[nodes.index(drive_mappings['LF'])],
					velocities[nodes.index(drive_mappings['LM'])],	
					velocities[nodes.index(drive_mappings['LB'])]
				  ]

		l_avg = sum(l_velos)/len(l_velos)

		r_velos = [ velocities[nodes.index(drive_mappings['RF'])],
					velocities[nodes.index(drive_mappings['RM'])],	
					velocities[nodes.index(drive_mappings['RB'])]
				  ]

		r_avg = sum(r_velos)/len(r_velos)

		#self.get_logger().info(f'{l_avg}, from {l_velos}')

		self.left_velo_buffer[self.ring_idx] = l_avg
		self.right_velo_buffer[self.ring_idx] = r_avg

		#Don't forget to increment the buffer:
		self.ring_idx = (self.ring_idx + 1) % (self.ring_len - 1)

		#Then calculate the moving average (list comprehension handles buffer filling):
		l_moving_avg = sum(self.left_velo_buffer) / len([v for v in self.left_velo_buffer if v != 0])
		r_moving_avg = sum(self.right_velo_buffer) / len([v for v in self.right_velo_buffer if v != 0])

		#Finally, compare each wheel's velocity with the side's moving average
		for wheel, node in drive_mappings.items():
	
			#note what wheel we're looking at:
			names[nodes.index(node)] = wheel

			#Get the current velocity
			velocity = velocities[nodes.index(node)]

			#Not sure the nicest way to format this, so doing this to determine RHS vs LHS:
			if 'R' in wheel: #Compare with RHS velo m_avg
				slip_ratio = abs(velocity - r_moving_avg)/abs(r_moving_avg + self.slip_bias)
			else: #Compare with LHS velo m_avg
				slip_ratio = abs(velocity - l_moving_avg)/abs(l_moving_avg + self.slip_bias)
			
			#self.get_logger().info(f"{velocity}")
			#self.get_logger().info(f"slip ratio {wheel}: {slip_ratio}")

			#Check if we are over the threshold
			if slip_ratio > self.slip_ratio_thresh:
				#We slippin
				slip_status[nodes.index(node)] = True
			#Otherwise do nothing since it's initialized to false

		#For now, just publish the slip status (seperately at a slower rate).
		#In the future (once this is throughouly tested), add some traction control

		self.slip_msg = TractionStatus()

		self.slip_msg.timestamp = msg.timestamp
		self.slip_msg.id = nodes
		self.slip_msg.names = names
		self.slip_msg.slipstate = slip_status

	def pub_msg(self):
		#For now, just publish the slip status (seperately at a slower rate).
		#In the future (once this is throughouly tested), add some traction control

		self.pub.publish(self.slip_msg)
		self.get_logger().info(f"published: {self.slip_msg.names}, {self.slip_msg.slipstate}")

#Create an entry point for the node:
def main(args=None):
	#init rclpy
	rclpy.init(args=args)

	#Create an instance of the node:
	slip_detector = SlipDetector()

	#Spin:
	rclpy.spin(slip_detector)

	#shutdown:
	rclpy.shutdown()


