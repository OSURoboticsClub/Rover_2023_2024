#Node which monitors canbus messages and periodically relays odrive message telemetry.

#Include rclpy, node, and numpy:
import rclpy
from rclpy.node import Node
import numpy as np

#Include the custom message definition:
from rover2_status_interface.msg import ODriveStatus
from std_srvs.srv import SetBool
#Include the relevant canbus libraries:
import struct #Used for handling byte arrays
import can

#List of odrive error codes
odrive_errors_by_code = { 
    0 : "NO ERROR",
    1 : "INITIALIZING",
    2 : "SYSTEM_LEVEL",
    4 : "TIMING_ERROR",
    8 : "MISSING_ESTIMATE",
    16 : "BAD_CONFIG",
    32 : "DRV_FAULT",
    64 : "MISSING_INPUT",
    256 : "DC_BUS_OVER_VOLTAGE",
    512 : "DC_BUS_UNDER_VOLTAGE",
    1024 : "DC_BUS_OVER_CURRENT",
    2048 : "DC_BUS_OVER_REGEN_CURRENT",
    4096 : "CURRENT_LIMIT_VIOLATION",
    8192 : "MOTOR_OVER_TEMP",
    16384 : "INVERTER_OVER_TEMP",
    32768 : "VELOCITY_LIMIT_VIOLATION",
    65536 : "POSITION_LIMIT_VIOLATION",
    16777216 : "WATCHDOG_TIMER_EXPIRED",
    33554432 : "ESTOP_REQUESTED",
    67108864 : "SPINOUT_DETECTED",
    134217728 : "BRAKE_RESISTOR_DISARMED",
    268435456 : "THERMISTOR_DISCONNECTED",
    1073741824 : "CALIBRATION_ERROR"
}


#Create a class for the node definition:
class OdriveTelemetry(Node):
	#Initialize the class
	def __init__(self, bus_name):
		super().__init__("odrive_telemetry")
	
		#Create timers for reading and republishing can data
		self.pub_timer = self.create_timer(0.05, self.pub_callback)
		self.can_timer = self.create_timer(0.05, self.read_can)

		#Create ros2 publisher
		self.pub = self.create_publisher(ODriveStatus, 'odrive_telem', 10)
                #Create ros2 service
		self.srv = self.create_service(SetBool, '/' + bus_name + '_clear_can', self.clear_can_callback)
		#establish the socketcan object:
		self.bus_name = bus_name
		self.bus = can.interface.Bus(bus_name, interface="socketcan")
		
		#Create data structures which will be filled in with the msg data:
		self.statuses = {}

		self.count = 0

		#Lastly, flush the canbus buffer with this gaurding loop.
		#Each time bus.recv() is called, it gets the next message in the buffer. 
		#This loop runs until the buffer is empty. 
		while not (self.bus.recv(timeout=0) is None): pass

	#Define a periodic callback for sending messages.
	def pub_callback(self):
		#Initialize the message
		msg = ODriveStatus()
		
		#Create the header/time to send
		msg.timestamp = self.get_clock().now().to_msg()
		msg.bus = self.bus_name

		#iterate over each node in the dictionary, adding it's data to the message
		for node, node_info in self.statuses.items():

			#Fill in the relevant telemetry data
			msg.id.append(node)
			
			msg.t_fet.append(node_info["fet_temp"])
			msg.t_motor.append(node_info["motor_temp"])
			msg.disarm_reason.append(node_info["disarm_reason"])
			msg.velocity.append(node_info["velocity"])
			msg.position.append(node_info["position"])
			msg.iq_set.append(node_info["iq_set"])
			msg.iq_measured.append(node_info["iq_measured"])
			msg.bus_voltage.append(node_info["bus_voltage"])
			msg.bus_current.append(node_info["bus_current"])
		
		#Send the message:
		#self.get_logger().info(f"{self.statuses}")
		#self.get_logger().info(f"can msgs read: {self.count}")
		self.count = 0
		self.pub.publish(msg)

	def clear_can_callback(self, request, response):
            nodes = [1,2,3,4,5]
            #If wanting to clear arm bus
            if(self.bus_name == "can1"):
                nodes = [1,2,3,4,5,6]
            for node_id in nodes:
                self.bus.send(can.Message(
                arbitration_id=(node_id << 5 | 0x18), # 0x0d: Clear_errors
                data=struct.pack('<I', 1), 
                is_extended_id=False
                ))

                self.bus.send(can.Message(
                arbitration_id=(node_id << 5 | 0x07), # Set axis state
                data=struct.pack('<I', 8), # 8: closed loop control
                is_extended_id=False
                ))
            response.success = True;
            response.message = self.bus_name + " cleared"
            
            return response
            
	#Define a callback for watching can messages:
	def read_can(self):
		#self.get_logger().info("starting to read msgs")
	
		#Get a buffer of stored messages and iterate over it
		can_msgs = self.get_can_buffer()

		for can_msg in can_msgs:

			#Don't do anything if a None message gets through
			if can_msg == None:
				continue

			#Masks for getting the command and node ids
			node_mask = (1 << 6) - 1 
			cmd_mask = (1 << 5) - 1

			#First pull out and save the node ID and command ID:
			node_id = (can_msg.arbitration_id >> 5) & node_mask
			cmd_id = can_msg.arbitration_id & cmd_mask
				
			#Probably should check for the RTR bit in case someone specifically requests data while this is running
			#if can_msg.rtr: #This means the message is a request -> no data
			#	continue

			#Use match with the command id to unpack the message correctly:
			result = {} #Assign this an empty dict in case the match falls thru
			match cmd_id:
				case 0x01: #Heartbeat
					pass	
				case 0x03: #Error/Disarm Reason codes
					active_error, disarm_reason = struct.unpack('<II', bytes(can_msg.data))
					#self.get_logger().info(f'disarm_reason is {disarm_reason}')
					result = {"disarm_reason" : odrive_errors_by_code[disarm_reason]} #integers are for suckers

				case 0x09: #Encoder Estimate of Position/Velocity
					pos_estimate, vel_estimate = struct.unpack('<ff', bytes(can_msg.data))
					#self.get_logger().info(f'position: {pos_estimate}')
					result = {"position" : pos_estimate,
							  "velocity" : vel_estimate}
				
				case 0x14: #Q Axis motor current set/measured
					iq_set, iq_measured = struct.unpack('<ff', bytes(can_msg.data))
					result = {"iq_set" : iq_set,
							  "iq_measured" : iq_measured}
					
				case 0x15: #Temperature monitoring: FET/motor Temp
					fet_temp, motor_temp = struct.unpack('<ff', bytes(can_msg.data))
					result = {"fet_temp" : fet_temp,
							  "motor_temp" : motor_temp}
					
				case 0x17: #Bus Voltage/Bus Current
					bus_voltage, bus_current = struct.unpack('<ff', bytes(can_msg.data))
					result = {"bus_voltage" : bus_voltage,
							  "bus_current" : bus_current}

				#case 0x1c: #Torque Target/Estimate
				#case 0x1d: #Electrical/Mechanical Power

			#Then update the dictionary for the corresponding node:
			#Try and except here lets us handle an arbitrary number of node IDs
			try:
				self.statuses[node_id].update(result)
			
			except:
				#If we haven't seen a node yet (no dictionary to update),
				# then we should make a complete empty dict
				self.statuses.update({node_id : {
												"disarm_reason" : "NO ERROR",
												"position" : 0.0,
												"velocity" : 0.0,
												"iq_set" : 0.0,
												"iq_measured" : 0.0,
												"fet_temp" : 0.0,
												"motor_temp" : 0.0,
												"bus_voltage" : 0.0,
												"bus_current" : 0.0 
												}})
				self.statuses[node_id].update(result)
		
        #Count the messages processed between publishing for curiousity.	
		self.count += len(can_msgs)

	#Abstraction for getting all can messages currently in the buffer:
	def get_can_buffer(self):
	
		#Return a max of 1000 msgs, that way we don't miss publishing
		max_return_msgs = 1000

		can_msgs = []
		
		msg_count = 0

		while True:
			#Check max msgs first (that way we don't lose a msg)
			if msg_count == max_return_msgs-1:
				break
		
			#Read the msg
			can_msg = self.bus.recv(timeout=0)
		
			#We get to the last msg if reading it returns None:
			if can_msg == None:
				break

			#Add them to a list and count the number:
			#can_msgs[msg_count] = can_msg
			can_msgs.append(can_msg)
			msg_count += 1

		#Return list of msgs
		return can_msgs

#Used for ODrives on the drivetrain (can0) network:
def drivetrain_telem(args = None):
	#Init rclpy:
	rclpy.init()

	#Create an instance of our node:
	odrive_telem = OdriveTelemetry("can0")

	rclpy.spin(odrive_telem)
	
	#Shut down in case this ever gets run:
	rclpy.shutdown()

#Used for ODrives on the Arm (can1) network:
def arm_telem(args = None):
	rclpy.init()
	odrive_telem = OdriveTelemetry("can1")
	rclpy.spin(odrive_telem)
	rclpy.shutdown

