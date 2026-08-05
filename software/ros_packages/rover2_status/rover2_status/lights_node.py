"""
Gripper Control Node
DAM Robotics
Authors: Jared Northrop
Year: 2526


Control tower lights. 

Notes
-----

"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rover_arm_control_interface.action._gripper_control import GripperControl
from sensor_msgs.msg import Joy
import can 
import struct
from time import time, sleep

from std_msgs.msg import String

class GripperCanControl(Node):

    def __init__(self):
        super().__init__('gripper_can_control')

        self.last_message_time = time()
        
        self.declare_parameter('joy_publish_rate', 30)
        self.declare_parameter('can', "can0")
        self.joy_publish_rate = self.get_parameter('joy_publish_rate').value
        self.can_network = self.get_parameter('can').value

        self.publish_rate = 100 #[hz]
        #odrive params
        self.node_id = 6
        self.laser_pin = 10
        self.lights_pin = 9

        #joy Mappings
        self.open_button = 1 
        self.close_button = 2
        self.light_button = 11
        self.laser_button = 12 

        #states
        self.lights = False 
        self.lightsChanged = False
        self.rgb_led = False
        self.laserChanged = False

        #setup can
        self.bus = can.interface.Bus(channel=self.can_network, bustype='socketcan')
        self.can_timer = self.create_timer(0.005, self.clear_can_buffer)
        while not (self.bus.recv(timeout=0) is None): pass
        #set up joy
        self.create_subscription(Joy, '/joy', self.joy_callback_v2, 1)

    def joy_callback_v2(self, msg):
        """Handles joy inputs. 

        This function takes controller inputs and changes the state of the node. Controller states are as 
        follows: 0 = Idle/Stopped, 1 = open, 2 = close.  Light and laser states are simply true for on and 
        false for off. 
        """
        self.last_message_time = time()
        buttons = msg.buttons
        if buttons[self.light_button]:
            if not self.lightsChanged:
                self.lights = not self.lights
                self.lightsChanged = True
                self.toggle_lights(state=self.lights)
        else:
            self.lightsChanged = False

        if buttons[self.laser_button]:
            if not self.laserChanged:
                self.rgb_led = not self.rgb_led
                self.laserChanged = True
                self.toggle_rgb_led(state=self.rgb_led)
        else:
            self.laserChanged = False

    def toggle_lights(self, state=False):
        """ Send a can command to toggle lights."""
        if state:
            try:
                self.bus.send(can.Message(
                    arbitration_id=(self.node_id << 5 | 0x01), # 
                    data=struct.pack('<B', 1), # Bool for toggle
                    is_extended_id=False
                ))
            except:
                self.get_logger().info("CAN Buffer full")
        else:
            try:
                self.bus.send(can.Message(
                    arbitration_id=(self.node_id << 5 | 0x01), # 
                    data=struct.pack('<B', 0), # Bool for toggle
                    is_extended_id=False
                ))
            except:
                self.get_logger().info("CAN Buffer full")

    def toggle_rgb_led(self, state=False):
        if state:
            try:
                self.bus.send(can.Message(
                    arbitration_id=(self.node_id << 5 | 0x05), # LED White
                    data=struct.pack('<B', 1), # Bool (doesn't matter what)
                    is_extended_id=False
                ))
            except:
                self.get_logger().info("CAN Buffer full")
        else:
            try:
                self.bus.send(can.Message(
                    arbitration_id=(self.node_id << 5 | 0x05), # LED White
                    data=struct.pack('<B', 0), # Bool (doesn't matter what)
                    is_extended_id=False
                ))
            except:
                self.get_logger().info("CAN Buffer full")

    def clear_can_buffer(self):
        while not (self.bus.recv(timeout=0) is None): pass        

    #Define a callback for watching can messages:
    def read_can(self):
        """Processes incoming can messages.

        This function reads a buffer of can messages sent on the bus by odrives and pulls out important 
        command values. Two commands are searched for: Encoder Estimate and Motor Current (get_IQ). 
        Encoder estimate contains the measured position and velocity while motor current contains the 
        measured motor current and setpoint. 
        """
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
            if node_id == self.node_id:
            #Use match with the command id to unpack the message correctly:
                result = {} #Assign this an empty dict in case the match falls thru
                match cmd_id:
                    case 0x01: #Heartbeat
                        pass	
                    case 0x09: #Encoder Estimate of Position/Velocity
                        pos_estimate, vel_estimate = struct.unpack('<ff', bytes(can_msg.data))
                        #self.get_logger().info(f'position: {pos_estimate} | velocity: {vel_estimate}')
                        self.measured_pos = pos_estimate
                        self.measured_vel = vel_estimate
                    
                    case 0x14: #Q Axis motor current set/measured
                        iq_set, iq_measured = struct.unpack('<ff', bytes(can_msg.data))
                        self.measured_current = iq_measured

                        #self.get_logger().info(f"Current: {iq_measured}, Set: {iq_set}")

                    case 0x1c: #Torque Target/Estimate
                        torque_set, torque_measured = struct.unpack('<ff', bytes(can_msg.data))
                        self.measured_torq = torque_measured
                        self.feedback_torq_setpoint =torque_set
                        #self.get_logger().info(f"Torque: {torque_measured} | Torque Set point = {torque_set}") 
                    case 0x05: #Tx Transmission
                        self.get_logger().info(f"Endpoint ID: {can_msg.data[1]} | Transmission: {can_msg.data[4:]}")
                    case 0x06: #Address reply
                        self.get_logger().info(f"Node ID: {can_msg.data[0]} | Serial Number: {can_msg.data[1:7]} | Connection ID: {can_msg.data[7]}")

    #Abstraction for getting all can messages currently in the buffer:
    def get_can_buffer(self):
        """Stores can messages in a buffer.'

        This function reads and stores can messages to a buffer for asynchronus parsing. Stores a maximum of
        1000 messages at a time. 

        Returns
        -------
            can_msgs : list
                The buffer of can messages. 
        """

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

def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperCanControl()

    rclpy.spin(gripper_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
