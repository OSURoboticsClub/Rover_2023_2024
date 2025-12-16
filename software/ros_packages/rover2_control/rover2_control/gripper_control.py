"""
Gripper Control Node
DAM Robotics
Authors: Jared Northrop
Year: 2425


Gripper control node for grippers using a single odrive motor controllers. Odrive firmware is expected to be 
on 0.6.10 (Requires changing all 0x04 can messages for different firmware. See Odrive docs on getting id from 
flat_endpoints.json). 

Notes
-----
Position control mode for timer and joy callbacks (v1) is unstable and can cause the odrive to error. Velocity
mode should be used for these callbacks

Timer and joy callbacks v2 are untested but should be a cleaner way to handle controls. Joy callback v2 only 
sets the current control state while the timer callback v2 handles control logic and sending commands to can 
bus. 

Extracting ID from flat endpoints needs to be implemented (not critical). 

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import can 
import struct
from time import time, sleep

from std_msgs.msg import String


class GripperCanControl(Node):

    def __init__(self):
        super().__init__('gripper_can_control')

        self.last_message_time = time()
        
        self.declare_parameter('is_position_control', False)
        self.declare_parameter('joy_publish_rate', 30)
        self.declare_parameter('direction', -1)
        self.declare_parameter('flat_endpoints', "")
        self.declare_parameter('can', "can1")
        self.is_position_control = self.get_parameter('is_position_control').value
        self.joy_publish_rate = self.get_parameter('joy_publish_rate').value
        self.direction = self.get_parameter('direction').value #direction odrive closes
        self.flat_endpoints = self.get_parameter('flat_endpoints').value #path to flat_enpoints.json
        self.can_network = self.get_parameter('can').value

        self.nu = 6 #scaling dx value Should be moved there and replaced with gear ratio
        self.publish_rate = 100 #[hz]
        #odrive params
        self.node_id = 6
        self.axis = 0
        self.laser_pin = 10
        self.lights_pin = 9

        #limits
        self.vel_limit = 60.0 #[rev/s]
        self.vel_ramp_rate = 200.0 #[rev/s^2]
        self.current_limit = 6.0 # [A]
        self.accel_limit = 100.0 #[rev/s^2]
        self.deccel_limit = 100.0 #[rev/s^2]
        self.vel_scale = 100 #Scales velocity feedforward for position mode. See Odrive Docs Set_Position
        self.torq_scale = 1000 #Scales torque feedforward for position mode. See Odrive Docs Set_Postion
        self.filter_bandwidth = 7 #effectively responsiveness of filtered position (ie acceleration)

        #Homing Params
        self.home_button = 10
        self.found_home = False
        self.is_homed = False
        self.home_current_threshold = 4.0 #[A]
        self.home_vel = 20.0 #[rev/s]
        self.home_pos = 0.0 #[rev]
        self.home_offset = self.direction * 50.0 # [rev] offset homepostion back so home position isn't at a hardstop

        #setpoints
        self.vel = 0.0 #[rev/s] Velocity to send (Timer_callback)

        self.vel_setpoint = 60.0 #[rev/s] Moving velocity setpoint
        self.torq_setpoint = 0.03124 #[Nm] Holding Torque to Send
        self.pos_setpoint = 0.0 #[rev] Position to send
        self.current_threshold = 4.0 #[A] Current threshold to change to torque mode
        self.dx = self.vel_limit / self.publish_rate * self.nu #unsure how to tune this Needs to be based on velocity limit and publish rate

        #Measured values
        self.measured_current = 0.0 #[A]
        self.measured_pos = 0.0 #[rev]
        self.measured_vel = 0.0 #[rev/s]
        self.measured_torq = 0.0 #[Nm]
        self.feedback_torq_setpoint = 0.0 #[Nm] Odrives can feedback setpoint

        #odrive mode
        self.mode = 0 #current mode see set_mode() for Details
        self.torq_timeout = 0.0

        #joy Mappings
        self.open_button = 1 
        self.close_button = 2
        self.light_button = 11
        self.laser_button = 12 

        #joy states
        self.controller_state = 0 #controller state. See timer_callback_v2/joy_callback_v2 for Details
        self.lights = False 
        self.laser = False

        #setup can
        self.bus = can.interface.Bus(channel=self.can_network, bustype='socketcan')
        self.can_timer = self.create_timer(0.005, self.read_can)
        while not (self.bus.recv(timeout=0) is None): pass
        #set up joy
        self.create_subscription(Joy, '/joy', self.joy_callback_v2, 1)

        #create time out timer
        self.timer = self.create_timer(0.015, self.timer_callback_v2)

        #Initialize Odrive and Gripper position 
        self.get_logger().info("Starting Setup of Odrive")
        self.setup_controller()

    def timer_callback(self):
        """This function sends can commands to the odrive at consistent frequency (required for velocity mode). 
        It also handles no controller input and ensuring the motor reaches desired set points. 
        
        """
        #complete Homing
        if not self.is_homed:
            #self.get_logger().info("sending vel")
            if not self.found_home:
                self.set_mode(2)
                self.send_velocity(self.home_vel)
            elif abs(self.measured_pos - self.pos_setpoint) > 0.02 and self.measured_vel < 0.01:
                self.send_position(self.home_pos)
            
            
        #timeout period of half sec
        elif self.is_position_control:
            # if self.current > self.current_threshold:
            #     self.send_mode(1)
            if self.mode == 1:
                if abs(self.measured_vel) > 0.5 and abs(-self.torq_setpoint - self.feedback_torq_setpoint) < 0.0001: 
                    self.get_logger().info(f"Velocity Measured: {self.measured_vel}")
                    #self.set_mode(2)
                    self.send_torque(0.0)
                    self.set_mode(0)
                elif abs(-self.torq_setpoint - self.feedback_torq_setpoint) > 0.0001:
                    self.get_logger().info(f"current: {self.measured_current} |Current Torque Setpoint {self.feedback_torq_setpoint} | Wanted Torque Setpoint {self.torq_setpoint}")
                    self.send_torque(-self.torq_setpoint)
            #self.get_logger().info(f"Current: {self.current}")
            #self.get_logger().info(f"Set Position: {self.pos_setpoint}")
            elif self.mode == 4:
                self.send_position(pos=self.pos_setpoint, vel_ff=int(self.vel))
        else:
            if abs(self.last_message_time - time()) > 0.5 and self.mode != 1:
                #self.get_logger().info("No inputs")
                if self.mode != 3:
                    self.set_mode(3)
                    self.send_position(self.measured_pos)
                elif abs(self.measured_pos - self.pos_setpoint) > 0.02 and self.measured_vel < 0.01:
                    self.send_position(self.pos_setpoint)
            #handle going to torque mode
            elif self.mode == 1:
                self.get_logger().info(f"current: {self.measured_current} |Current Torque Setpoint {self.feedback_torq_setpoint} | Wanted Torque Setpoint {self.torq_setpoint}")
                if abs(self.measured_vel) > 0.5 and abs(-self.torq_setpoint - self.feedback_torq_setpoint) < 0.0001: 
                    self.get_logger().info(f"Velocity Measured: {self.measured_vel}")
                    #self.set_mode(2)
                    self.send_torque(0.0)
                    self.set_mode(0)
                elif abs(-self.torq_setpoint - self.feedback_torq_setpoint) > 0.0001:
                    self.send_torque(-self.torq_setpoint)
            #Handle Velocity Mode
            elif self.mode == 2:
                self.send_velocity(self.vel)
            #Handle Position mode:
            elif self.mode == 3:
                if abs(self.measured_pos - self.pos_setpoint) > 0.02 and self.measured_vel < 0.01:
                    self.send_position(self.pos_setpoint)
            elif self.mode == 4:
                if abs(self.measured_pos - self.pos_setpoint) > 0.02 and self.measured_vel < 0.01:
                    self.send_position(self.pos_setpoint)

    def timer_callback_v2(self):
        """This function handles commanding the current state set by the joy callback to motor controllers.

        This function handles sending commands to the can bus for each gripper state. The Gripper operates in 
        three states: opening, closing, stopped. Two modes of controlling the gripper can be used: velocity 
        and position. Upon startup the node will first home the gripper in which case operator commands are 
        ignored. Commands are sent to the can bus at a frequency of ~100hz. The functions handles controller 
        timeout and holding an object. 
        """
        #Homing
        #self.get_logger().info(f"b measured current: {self.measured_current}")
        if not self.is_homed:
            if not self.found_home:
                if self.measured_current > self.home_current_threshold:
                    self.get_logger().info(f"measured current: {self.measured_current}")
                    self.home_pos = self.measured_pos - self.direction * self.home_offset
                    self.found_home = True
                    self.set_mode(3)
                    self.send_position(self.home_pos)
                else:
                    self.set_mode(2)
                    self.send_velocity(self.home_vel)
            elif abs(self.measured_pos - self.home_pos) < 0.08: 
                self.is_homed = True
                self.get_logger().info("Homed")
            elif abs(self.measured_pos - self.pos_setpoint) > 0.02 and self.measured_vel < 0.01:
                self.send_position(self.home_pos)
        #Position Control Mode
        elif self.is_position_control:
            #Time out
            if abs(time() - self.last_message_time) > 0.25:
                self.controller_state = 0
            match self.controller_state:
                #No input state
                case 0:
                    pass
                #open input state
                case 1:
                    #don't go past home
                    new_setpoint = self.pos_setpoint - self.direction * self.dx
                    if self.direction * (self.home_pos - new_setpoint) <= 0:
                        self.set_mode(4)
                        self.send_position(new_setpoint)
                #close input state
                case 2:
                    #Handle grasping object
                    if abs(self.measured_current) < self.current_threshold: 
                        self.set_mode(4)
                        self.send_position(self.pos_setpoint + self.direction * self.dx)
                    else:
                        # Torq setpoint may not be high enough for measured current > current threshold at all times. 
                        self.set_mode(4)
                        self.send_position(self.measured_pos, torq_ff=int(self.torq_setpoint * 1000))
        #Velocity Control mode
        else:
            #Time out
            if abs(time() - self.last_message_time) > 0.25: #Ideally set to some scalar bigger than joy_publish rate
                self.controller_state = 0
            match self.controller_state:
                #No input state
                case 0:
                    if self.mode != 1:
                        self.set_mode(2)
                        #don't continuously send 0
                        if abs(self.measured_vel) > 0.001:
                            self.send_velocity(0.0)
                #open input state
                case 1:
                    #don't go past home
                    if self.direction * (self.home_pos - self.measured_pos) <= 0:
                        self.set_mode(2)
                        self.send_velocity(-self.direction * self.vel_setpoint)
                    #need to ensure vel_ramp_rate is high enough not to hit the hardstop
                    else:
                        self.get_logger().info("Past Home")
                        self.set_mode(2)
                        self.send_velocity(0.0)
                #close input state
                case 2:
                    #Handle Current
                    if self.mode == 1:
                        if abs(self.measured_vel) > 10.0 and abs(-self.torq_setpoint - self.feedback_torq_setpoint) < 0.0001 and abs(self.measured_current) < 2.0 and abs(self.torq_timeout - time()) < 0.25: 
                            self.get_logger().info(f"Velocity Measured: {self.measured_vel} | current: {self.measured_current}")
                            self.send_torque(0.0)
                            self.set_mode(2)
                            self.send_velocity(0.0)
                        elif abs(-self.torq_setpoint - self.feedback_torq_setpoint) > 0.0001:
                            self.send_torque(-self.torq_setpoint)
                    elif abs(self.measured_current) > self.current_threshold:
                        self.get_logger().info("at torque")
                        self.torq_timeout = time()
                        self.set_mode(1)
                        if abs(-self.torq_setpoint - self.feedback_torq_setpoint) > 0.0001:
                            self.send_torque(-self.torq_setpoint)
                    else:
                        self.set_mode(2)
                        self.send_velocity(self.direction * self.vel_setpoint)

    def joy_callback(self, msg):
        """Handles controller inputs. 
        """
        self.last_message_time = time()
        buttons = msg.buttons
        #wait for homing to complete
        if self.is_homed:
            if self.is_position_control:
                self.vel = int(0)
                if buttons[self.open_button]:
                    self.set_mode(4)
                    if self.pos_setpoint >= self.home_pos:
                        self.pos_setpoint = self.home_pos
                        self.vel = int(0)
                    else:
                        self.pos_setpoint = self.pos_setpoint + self.dx
                        #self.vel = int(self.vel_setpoint*self.vel_scale)
                    
                elif buttons[self.close_button]:
                    self.get_logger().info(f"Joy current: {self.measured_current}")
                    if abs(self.measured_current) < self.current_threshold and self.mode != 1:
                        self.set_mode(4)
                        self.pos_setpoint = self.pos_setpoint - self.dx
                        #self.vel = int(-self.vel_setpoint*self.vel_scale)
                    
                    else:
                        self.get_logger().info("At Threshold")
                        self.pos_setpoint = self.measured_pos
                        self.set_mode(1)
            else:
                if buttons[self.open_button]: 
                    #ensure we don't go past fully open position
                    if self.measured_pos >= self.home_pos:
                        self.set_mode(3)
                        self.pos_setpoint = self.home_pos
                    #set velocity mode and setpoint
                    else:
                        self.set_mode(2)
                        self.vel = self.vel_setpoint

                elif buttons[self.close_button]: 
                    #Check if grasping an object or if the gripper is closed and Hold Torque
                    self.get_logger().info(f"current: {self.measured_current}")
                    if abs(self.measured_current) > self.current_threshold:
                        self.get_logger().info("setting torque mode")
                        self.set_mode(1)
                    elif self.mode != 1: 
                        self.set_mode(2)
                        self.vel = -self.vel_setpoint
                # Stop moving if not in torque control
                elif self.mode == 2:
                    self.set_mode(2)
                    # direction = 1 if self.current_vel >= 0 else -1
                    # if self.current_vel > self.vel_setpoint - 10:
                    #     self.pos_setpoint = self.current_pos + direction * self.dx
                    # else:
                    #     self.pos_setpoint = self.current_pos
                    # self.vel = int(0)

                    self.vel = 0.0

    def joy_callback_v2(self, msg):
        """Handles joy inputs. 

        This function takes controller inputs and changes the state of the node. Controller states are as 
        follows: 0 = Idle/Stopped, 1 = open, 2 = close.  Light and laser states are simply true for on and 
        false for off. 
        """
        self.last_message_time = time()
        buttons = msg.buttons
        if buttons[self.open_button]:
            self.controller_state = 1
        elif buttons[self.close_button]:
            self.controller_state = 2
        else:
            self.controller_state = 0
        if buttons[self.light_button]:
            self.lights = not self.lights
            self.send_gpio(self.lights_pin, self.lights)
        if buttons[self.laser_button]:
            self.laser = not self.laser
            self.send_gpio(self.laser_pin, self.laser)
        if buttons[self.home_button]:
            self.get_logger().info("Re-Homing")
            self.found_home = False
            self.is_homed = False
            

    def setup_controller(self):
        """Initialize controller.
        """
        #initialize control mode
        self.set_mode(self.mode)
        #self.home()
        self.get_logger().info("Finished Setup")

    def home(self):
        """Find home position by hitting hardstop limit.

        To home the motor is moved in the open direction at a constant velocity until a current threshold is 
        reached. The current threshold indicates hitting a hardstop limit which is considered the home 
        position once an offset is applied. The offset adds a buffer so that the hardstop can't be hit. 
        """
        self.get_logger().info("Start Homing Sequence")
        self.set_mode(2) #enable closed loop ramped velocity mode
        while True:
            #self.get_logger().info("homing")
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.measured_current > self.home_current_threshold:
                self.home_pos = self.measured_pos - self.direction * self.home_offset
                self.found_home = True
                self.set_mode(3)
                self.send_position(self.home_pos)
                break
            sleep(0.01)

        while True: 
            rclpy.spin_once(self, timeout_sec=0.01)
            if abs(self.measured_pos - self.home_pos) < 0.04:
                self.is_homed = True
                break
            sleep(0.01)
        self.get_logger().info(f"Homed, Position: {self.home_pos}")
        self.last_message_time = time()

    def send_torque(self, torq):
        """Send a torque set point through can.
        
        Parameters
        ----------
            torq : float
                A torque set point. 
        """
        try:

            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x0e),
                data=struct.pack('<f', torq),
                is_extended_id=False
            ))
            self.get_logger().info(f"sent torque: {torq}")
        except:
            self.get_logger().info("CAN Buffer full")

    def send_velocity(self, vel, torq_ff = 0.0):
        """Send a velocity set point through can.
        
        Parameters
        ----------
            vel : float
                A velocity setpoint.
            torq_ff : float, optional
                The torque feed forward value.
        """
        try:
        
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
                data=struct.pack('<ff', vel, torq_ff), # 1.0: velocity, 0.0: torque feedforward
                is_extended_id=False
            ))
        except:
            self.get_logger().info("CAN Buffer full")
        
        #self.get_logger().info(f"Sending Velocity: {vel}")  
        
    def send_position(self, pos, vel_ff = 0, torq_ff = 0):
        """Send a position set point through can.
        
        Parameters
        ----------
            pos : float
                A position set point. 
            vel_ff : int, optional
                The velocity feed forward value in 1/1000 rev/s.
            torq_ff : int, optional
                The torque feed forward value in 1/1000 Nm.
        """
        try:
            self.bus.send(can.Message(
                arbitration_id = (self.node_id << 5 | 0x0c),
                data = struct.pack('<fhh', pos, vel_ff, torq_ff),
                is_extended_id = False
            ))
        #self.get_logger().info(f"sent position: {pos}")
            self.pos_setpoint = pos
        except:
            self.get_logger().info("CAN Buffer full")
    
    def send_gpio(self, pin, state):
        """Sends a can message to change a gpio pins state

        Parameters
        ----------
            pin : uint32
                The gpio pin to control.
            state : bool
                The state to set. 
        """
        if True:
            #try:
            self.bus.send(can.Message(
                arbitration_id = (self.node_id << 5 | 0x04),
                data = struct.pack('<BHB', (7 << 2 | 1), 653, 1),
                is_extended_id = False
            ))
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | 0x04),
                data=struct.pack('<BBI?', (7 << 2 | 3), (1 << 4 | 1), pin, state),
                is_extended_id=False
            ))
            #except:
                #self.get_logger().info("CAN Buffer full")
    def set_mode(self, mode):
        """Sets the desired control mode. 
        
        Parameters
        ----------
            mode : {0, 1, 2, 3, 4}
                The desired control mode {idle, closed loop(CL) torque, CL ramped velocity , CL trapezoidal 
                trajectory, CL filtered position}
        """
        if self.mode != mode:
            self.get_logger().info(f"mode: {mode}")
            try:
                match mode:
                    case 0:
                        #Set Cloosed Loop control
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                            data=struct.pack('<I', 1), # 8: AxisState.IDLE
                            is_extended_id=False
                        ))
                        self.mode = 0

                    case 1:
                        #Set Cloosed Loop control
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                            is_extended_id=False
                        ))
                        ##set Torq control and passthrough
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x0b), 
                            data=struct.pack('<II', 1,1),
                            is_extended_id=False
                        ))
                        self.mode = 1
                    case 2:
                        #Set Cloosed Loop control
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                            is_extended_id=False
                        ))
                        #Set velocity ramp control mode
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x0b), 
                            data=struct.pack('<II', 2,2),
                            is_extended_id=False
                        ))

                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x04), 
                            data=struct.pack('<BHBf', 1, 403,0, self.vel_ramp_rate), #403 - 0.6.10, 396 - 0.6.9-1
                            is_extended_id=False
                        ))
                        self.mode = 2
                    case 3:
                        #Set Cloosed Loop control
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                            is_extended_id=False
                        ))
                        #set command mode position, input mode trap_traj
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x0b),
                            data=struct.pack('<II', 3, 5),
                            is_extended_id= False
                        ))

                        #Traj Velo Limit
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x11),
                            data=struct.pack('<f', self.vel_setpoint),
                            is_extended_id= False
                        ))

                        #trap accel limit
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x12),
                            data=struct.pack('<ff', self.accel_limit, self.deccel_limit),
                            is_extended_id= False
                        ))

                        #set vel and current limit
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x0f),
                            data=struct.pack('<ff', self.vel_limit, self.current_limit),
                            is_extended_id = False
                        ))
                        self.mode = 3
                    case 4: 
                        #Set Cloosed Loop control
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x07), # 0x07: Set_Axis_State
                            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
                            is_extended_id=False
                        ))
                        #set command mode position, input mode filtered_pos
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x0b),
                            data=struct.pack('<II', 3, 3),
                            is_extended_id= False
                        ))
                        #set vel and current limit
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x0f),
                            data=struct.pack('<ff', self.vel_limit, self.current_limit),
                            is_extended_id = False
                        ))
                        #set bandwidth
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x04), 
                            data=struct.pack('<BHBf', 1, 414,0, self.filter_bandwidth), 
                            is_extended_id=False
                        ))
                        #set feedforward scale 0.6.10 vel = 252 torq = 253
                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x04), 
                            data=struct.pack('<BHBI', 1, 252,0, self.vel_scale),
                            is_extended_id=False
                        ))

                        self.bus.send(can.Message(
                            arbitration_id=(self.node_id << 5 | 0x04), 
                            data=struct.pack('<BHBI', 1, 253,0, self.torq_scale), 
                            is_extended_id=False
                        ))
                        self.mode = 4
            except: 
                self.get_logger().info("failed to change mode: Can buffer full?")

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
