#!/usr/bin/env python3


# integrates all data from nodes and sends it to UI
#
# integrator.py
#
# Wyatt Boer & Osian Leahy


# Every Python node in ROS2 should include these lines. rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node. uwu
import rclpy
from rclpy.node import Node

import rcl_interfaces

# We're subscribed to MoveItLogs, NodesTopics and FilteredLog custom messages
from rover2_status_interface.msg import NodesTopics
from rover2_status_interface.msg import FilteredLog
from rover2_status_interface.msg import MoveItLogs
from rover2_status_interface.msg import ODriveStatus
from rover2_status_interface.msg import TractionStatus

# Our custom services
from rover2_status_interface.srv import NodeInfo
from rover2_status_interface.srv import LatencySize
from rover2_status_interface.srv import NodeLog

#We're using parameters:
from rclpy.parameter import parameter_value_to_python
#from rclpy.parameter_event_handler import ParameterEventHandler #Doesn't exist in Humble :(

#Import rich libraries for table generation:
from rich.live import Live
from rich.table import Table

class Integrator(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('integrator')

		#whether or not to use table display:
		self.table_display = True
	
		self.node_select_needs_updating = False

		# Create the subscribers
		#Subscribe to /moveit_logger, /nodetopiclisten, /log_filt, /odrive_telem, /slip_status
		self.node_topic_status = self.create_subscription(NodesTopics, 'nodetopiclisten', self.node_topic_callback, 10)
		self.node_logs = self.create_subscription(FilteredLog, 'log_filt', self.node_log_callback, 10)
		self.moveit_logs = self.create_subscription(MoveItLogs, 'moveit_logs',self.moveit_logger_callback,10)
		self.odrive_status = self.create_subscription(ODriveStatus, 'odrive_telem', self.odrive_status_callback, 10)
		self.slip_status = self.create_subscription(TractionStatus, 'slip_status', self.slip_callback, 10)

		#We have Six main data sets to store/display from the subscribed messages:
		#First is the Node and Topic lists:
		self.topic_names = []
		self.topic_statuses = []
		self.node_names = []
		self.node_statuses = []

		#Second is the Node Logs:
		self.log_stamp = self.get_clock().now().to_msg()
		self.log_name = ""
		self.log_level = 0
		self.log_msg = ""
		self.node_latency = ""
		
		#Third is our Node Topics/Services/Params:
		self.node_pubs = []
		self.node_subs = []
		self.node_params = []
		self.node_services = []
		
		#Fourth is our MoveIt data
		self.moveit_header = []
		self.moveit_name = []
		self.moveit_position = []
		self.moveit_velocity = []
		self.moveit_effort = []
		self.moveit_status = []

		#Fifth is the ODrive Statuses:
		self.odrive_timestamp = self.get_clock().now()
		self.bus = ""

		self.id = []
		self.disarm_reason = []
		self.t_fet = []
		self.t_motor = []
		self.velocity = []
		self.position = []
		self.iq_set = []
		self.iq_measured = []
		self.bus_voltage = []
		self.bus_current = []

		#Sixth is the Slip detection:
		self.slip_timestamp = self.get_clock().now()
		self.slip_id = []
		self.slip_names = []
		self.slipstate = []

		#Setup the service clients
		#We can select a node for log info, and get a list of a nodes pub/subbed topics, paramters, services 
		self.info_cli = self.create_client(NodeInfo, 'node_info')
		self.log_select_cli = self.create_client(NodeLog, 'node_log')

		#These service calls will be fed by node name/namespace parameters
		# The controlling node (unity in the future) can update this parameter to change the node info/log data sent.
		self.node_select = 'integrator'
		self.declare_parameter('node_select', self.node_select)
		self.namespace_select = '/'
		self.declare_parameter('namespace_select', self.namespace_select)
		self.odrive_select = 0
		self.declare_parameter('odrive_select', self.odrive_select)

		#We're gonna use a parameter callback to handle downstream updates:
		''' self.param_handler = ParameterEventHandler(self)
		self.callback_handles.append(self.param_handler.add_parameter_callback(
			parameter_name = 'node_select',
			node_name = 'integrator',
			callback = self.param_callback
		))
		
		self.callback_handles.append(self.param_handler.add_parameter_callback(
			parameter_name = 'namespace_select',
			node_name = 'integrator',
			callback = self.param_callback
		))
		
		self.callback_handles.append(self.param_handler.add_parameter_callback(
			parameter_name = 'odrive_select',
			node_name = 'integrator',
			callback = self.param_callback
		)) '''

		#Before exiting the init step, we want to know that the info/log select services are available, otherwise chaos:
		while not (self.info_cli.wait_for_service(timeout_sec=1) or self.log_select_cli.wait_for_service(timeout_sec=1)):
			self.get_logger().info('Waiting for a service to start')

	### Define our class Functions ###

	# This callback will be called every time that the nodetopiclisten topic is published to.
	# Just updates the class lists:
	def node_topic_callback(self, msg):
		#Quick logging message:
		self.get_logger().info('Recieved node/topic status list message')

		#Save the most recent data we get from our callback
		self.topic_names =  msg.topic_name 
		self.topic_statuses = msg.topic_status 
		self.node_names = msg.node_name 
		self.node_statuses = msg.node_status

	#This callback is called every time a new node_log message is published. 
	def node_log_callback(self, msg):
		# Sets current time to use to calculate node latency using log stamp later
		current_time = self.get_clock().now().nanoseconds
		
		#Quick Logging message:
		self.get_logger().info('Recieved Node log message')

		#Save the most recent log we got from our callback
		self.log_stamp = msg.stamp
		self.log_name = msg.name
		self.log_level = msg.level
		self.log_msg = msg.msg

		#TODO probably nicer update this to average the latency across last 5-10 values or smth instead of raw
		# Calculating latency (in seconds)
		self.node_latency = (current_time - msg.stamp.sec*10**9 - msg.stamp.nanosec)/(1*10**9)

	def moveit_logger_callback(self,msg):
		
		#Quick Logging message:
		self.get_logger().info('Recieved MoveIt log message')
		
		#https://moveit.picknik.ai/humble/api/html/status__codes_8h.html
		#reference for what status codes mean
		# dictionary of what msg codes mean
		status_mapping = {
		-1:'INVALID',
		0:'NO_WARNING',
		1:'DECELERATE_FOR_APPROACHING_SINGULARITY',
		2:'HALT_FOR_SINGULARITY',
  		3:'DECELERATE_FOR_COLLISION',
		4:'HALT_FOR_COLLISION',
		5:'JOINT_BOUND',
		6:'DECELERATE_FOR_LEAVING_SINGULARITY' 
		}
	
		#Save the most recent data we get from our callback	
		self.moveit_header = msg.header
		self.moveit_name = msg.name
		self.moveit_position = msg.position
		self.moveit_velocity = msg.velocity
		self.moveit_effort = msg.effort
		self.moveit_status = status_mapping[msg.status]
	
	def odrive_status_callback(self, msg):
		#Unpack the msg:
		#We will pare the data down in the generate table functions...

		self.odrive_timestamp = msg.timestamp
		self.bus = msg.bus

		self.id = msg.id
		self.disarm_reason = msg.disarm_reason
		self.t_fet = msg.t_fet
		self.t_motor = msg.t_motor
		self.velocity = msg.velocity
		self.position = msg.position
		self.iq_set = msg.iq_set
		self.iq_measured = msg.iq_measured
		self.bus_voltage = msg.bus_voltage
		self.bus_current = msg.bus_voltage
	
	def slip_callback(self, msg):
		#Unpack the msg, pare the data down later.
		self.slip_timestamp = msg.timestamp
		self.slip_id = msg.id
		self.slip_names = msg.names
		self.slipstate = msg.slipstate

	#If we need to update the selected node to introspect, do two service calls:
	#TODO: Is there a clean non-blocking way to do this better, or should we not care...
	def update_node_selection(self):
		
		#First build the requests
		self.info_request = NodeInfo.Request()
		self.info_request.node = self.get_parameter('node_select').get_parameter_value().string_value
		
		self.log_select = NodeLog.Request()
		self.log_select.enable = True
		self.log_select.hz = 1.0
		self.log_select.node = self.get_parameter('node_select').get_parameter_value().string_value

		#Then make the async calls and wait for them to return
		self.info_future = self.info_cli.call_async(self.info_request) #potential for race condition shennanigans here?
		self.log_future = self.log_select_cli.call_async(self.log_select)
		rclpy.spin_until_future_complete(self, self.info_future)
		rclpy.spin_until_future_complete(self, self.log_future) #probably don't need this

		#Unpack the returned info (nothing to unpack in the log response):
		info_response = self.info_future.result()
		self.node_pubs = info_response.pub_topics
		self.node_subs = info_response.sub_topics
		#self.node_params = info_response.parameters
		self.node_services = info_response.services

	#When the parameter changes, we should update the node selection downstream:
	'''def param_callback(self, parameter):
		#Quick logging message:
		self.get_logger().info(f'Parameter Changed: {parameter.name} = {parameter_value_to_python(parameter.value)}')

		#Prepare to call update_node_selection() since our node of interest changed:
		self.node_select_needs_updating = True
	'''

	#Creates a table of nodes and their dead/alive statuses
	def generate_node_status_table(self):
		#Generate the table headers:
		table = Table(title="Node Status")
		table.add_column("Node")
		table.add_column("Status")

		#fill in the table data:
		for node,status in zip(self.node_names,self.node_statuses):
			self.get_logger().info(f'row attempt: {str(node)}, {str(status)}')
			table.add_row(str(node),str(status))
		
		return table
	
	# Creates a table of MoveIt Logger info
	# TODO make sure table has all desired info + proper format and integrates properly with terminal UI 
	def generate_moveit_logger_table(self):

		# TODO make sure this actually creates the table how I want 
		#Generate the table headers:
		table = Table(title="MoveIt Logs")
		table.add_column("Joint Name")
		table.add_column("Position")
		table.add_column("Velocity")
		table.add_column("Effort")


		for name, position, velo, effort in zip(self.moveit_name,self.moveit_position, self.moveit_velocity, self.moveit_effort):
			table.add_row(str(name),str(position),str(velo),str(effort))

		table.add_row("Status", str(self.moveit_status))
		
		
		table.show_lines = True
		
		return table
	#Creates a table of node info: pubs/subs/params/services:
	def generate_node_info_table(self):
		#Layout each method as a row, unsure what this will do since they're diff. lengths
		table = Table(title="Node Communications")
		table.add_row("Subscribed Topics", *self.node_subs)
		table.add_row("Publishing Topics", *self.node_pubs)
		table.add_row("Services", *self.node_services)

		table.show_header = False
		table.show_lines = True

		return table

	#Creates a table of simplified node log info:
	def generate_node_log_table(self):
		
		#Dictionary of message levels:
		levels = {10:"DEBUG",
				  20:"INFO",
				  30:"WARN",
				  40:"ERROR",
				  50:"FATAL"}

		table = Table(title=f"{self.log_name} Log Info")
		table.add_row('time', f'{self.log_stamp.sec}.{self.log_stamp.nanosec}')
		table.add_row("level",levels.get(self.log_level))
		table.add_row('msg',self.log_msg)
		table.add_row('latency',str(self.node_latency))
		
		table.show_header = False
		table.show_lines = True

		return table

	#Gives an overview of all odrives on the selected can bus.
	def generate_odrive_overview_table(self):
		
		table = Table(title=f"ODrives on {self.bus}")
		table.add_row('Node IDs',*list(map(str,self.id)))
		table.add_row('Motor Current',*list(map(str,self.iq_measured)))
		table.add_row('Motor Temp',*list(map(str,self.t_motor)))
		table.add_row('Disarm Reason',*list(map(str,self.disarm_reason)))

		table.show_header = False
		table.show_lines = True

		return table

	#Gives a detailed view of a specific selected Odrive
	def generate_odrive_specific_table(self):
		
		#Get the node id from the parameter, and find the corresponding idx 
		node_id = self.get_parameter('odrive_select').get_parameter_value().integer_value
		
		#init the table:
		table = Table(title=f'ODrive {node_id} on {self.bus}')

		try:
			idx = self.id.index(node_id)
		except:
			#Return an empty table if idx is not available
			return table

		table.add_row("Position", str(self.position[idx]))
		table.add_row("Velocity", str(self.velocity[idx]))
		table.add_row("Motor Current", str(self.iq_measured[idx]))
		table.add_row("Motor Temp", str(self.t_motor[idx]))
		table.add_row("Bus Voltage", str(self.bus_voltage[idx]))
		table.add_row("Bus Current", str(self.bus_current[idx]))
		table.add_row("FET temp", str(self.t_fet[idx]))
		table.add_row("Disarm reason",str(self.disarm_reason[idx]))

		table.show_header = False
		table.show_lines = True

		return table		

	#Give an overview of all six wheels and wether or not they are slipping:
	def generate_slip_status_table(self):
	
		table = Table(title=f"Wheel Slip Status")

		table.add_row("Wheels", *list(map(str,self.slip_names)))
		table.add_row("Slipping", *list(map(str,self.slipstate)))

		table.show_header = False
		table.show_lines = True

		return table

	def generate_tables(self):
		status_table = self.generate_node_status_table()
		info_table = self.generate_node_info_table()
		log_table = self.generate_node_log_table()
		moveit_table = self.generate_moveit_logger_table()
		odrive_overview = self.generate_odrive_overview_table()
		odrive_specific = self.generate_odrive_specific_table()
		slip_table = self.generate_slip_status_table()

		grid = Table(box=None)
		grid.add_row(status_table,info_table,log_table,moveit_table)
		grid.add_row(odrive_overview,odrive_specific)
		grid.add_row(slip_table)

		grid.show_header = False
		grid.show_lines = False
		grid.show_edge = False

		return grid

# This is the entry point for the node.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# The ROS2 idiom is to encapsulate everything in a class derived from Node.
	integrate = Integrator()
	
	with Live(integrate.generate_tables(), refresh_per_second=15) as live:

		while rclpy.ok():
			#Poll the parameters we care about since we're on ros2 humble :/
			
			new_node = integrate.get_parameter('node_select').get_parameter_value().string_value
			new_namespace = integrate.get_parameter('namespace_select').get_parameter_value().string_value
			
			if (new_node != integrate.node_select):
				integrate.node_select = new_node
				integrate.node_select_needs_updating = True
			if (new_namespace != integrate.namespace_select):
				integrate.namespace_select = new_namespace
				integrate.node_select_needs_updating = True

			#Check if we need to update the node to introspect (runs the relevant service calls)
			if integrate.node_select_needs_updating:
				integrate.node_select_needs_updating = False
				integrate.update_node_selection()
		
			#Generate and update the tables we want to see:
			live.update(integrate.generate_tables())

			#Spin once each loop iteration
			rclpy.spin_once(integrate)

	#This never runs <3
	rclpy.shutdown()


