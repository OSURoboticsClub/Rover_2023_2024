import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage
from time import time, sleep
import math
import can
import struct

MAX_ACCEL = 0.05

GEAR_RATIO = 50
RPS_FACTOR = 4
#THIS SHOULD BE 2, 4.4 IS FOR THE SPEED LOL
#RPS_FACTOR = 4.4
BUS = can.interface.Bus("can0", interface="socketcan")

VEL_RAMP = GEAR_RATIO*RPS_FACTOR
LEFT_NODES = [0,2,4]
RIGHT_NODES = [1,3,5]
NODES = [0,1,2,3,4,5]

WATCHDOG_TIMEOUT = 3.0

class DriveCanControlNode(Node):
    def __init__(self):
        super().__init__('drive_can_control_node')

        #Initialize class variables for command velocities
        #These are unity/should be constrained to [-1,1]
        #TODO: Fix drivetrain control inputs so we can command actual velocity in m/s :eyeroll:
        self.linear_velocity = 0
        self.angular_velocity = 0

        #Clear the CAN message buffer at the beginning so we don't get full buffer errors 
        while not (BUS.recv(timeout=0) is None): pass
 
        #Setup all the ODrives
        self.setup_controller()

        #Groundstation Subscription
        #TODO: Just make these the Default Twist Message types?? :skull:
        self.groundstation_sub = self.create_subscription(
            DriveCommandMessage,
            '/command_control/ground_station_drive',  # Topic where twist messages are published
            self.groundstation_drive_command_callback,
            10
        )

        #Iris subscription for Taranis
        self.iris_sub = self.create_subscription(
            DriveCommandMessage,
            '/command_control/iris_drive',  # Topic where twist messages are published
            self.iris_drive_command_callback,
            10
        )

        #Run the drive control at 50hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        #Class variable for the watchdog timeout
        self.last_message_time = time()


    #Function for initializing all ODrive S1s.
    #Iterates over each node and sets the axis state, control mode (velocity), and velocity ramp Limit
    def setup_controller(self):
        for node_id in NODES:
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
            ))
          
            #Set velocity ramp control mode
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0b), 
            data=struct.pack('<II', 2,2),
            is_extended_id=False
            ))

            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x04), 
            data=struct.pack('<BHBf', 1,403,0,VEL_RAMP),
            is_extended_id=False
            ))


    #This callback is called as a 50hz loop, as configured on node init.
    def timer_callback(self):

        #Watchdog to make sure the drive doesn't spin out of control if connection is lost/messages stop being recieved
        if time() >= self.last_message_time+2:    
            self.linear_velocity = 0.0  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = 0.0  # Right joystick horizontal axis (turning)

        #This handles drivetrain saturation, control mixing, and sending messages over CAN
        self.send_drive_commands()

        #self.get_logger().info(f"Time: {time()}, Last Message Time: {self.last_message_time}")
        #self.get_logger().info(f"linear vel: {self.linear_velocity} | angular velocity: {self.angular_velocity}")


    #Callback function for Groundstation Drive commands.
    def groundstation_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        self.get_logger().info(f"Recieved: Lin Vel: {msg.drive_twist.linear.x}, ang vel: {msg.drive_twist.angular.z}")
        if msg.controller_present:
            #Update the desired velocities
            self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)         self.send_drive_commands() 
            
            self.last_message_time = time() #Only update the watchdog timer if we recieve a message and a controller is present


    #Callback Function for Drive commands from Iris, which recieves commands over SBUS from Taranis   
    def iris_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        if msg.controller_present:

            self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)
            self.send_drive_commands()
            # You may want to scale these velocities to suit your needs (e.g., make them faster/slower)
            self.last_message_time = time()


    #This computes and sends the drive commands
    def send_drive_commands(self):
        
        #Get the left, right command velocities
        left_velocity, right_velocity = self.compute_drive_sides()

        #Logic for sending velocity through can HERE
        for node_id in LEFT_NODES:
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', left_velocity, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
            ))

        for node_id in RIGHT_NODES:
            BUS.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', right_velocity, 0.0), # 1.0: velocity, 0.0: torque feedforward
            is_extended_id=False
            ))


    #Function to compute the actual Differential drive commanded speeds from the forward/angular velocity intput
    def compute_drive_sides(self):

        #Compute the left and right speeds from commanded values 
        left_command = -1 * (self.linear_velocity - self.angular_velocity) 
        right_command = (self.linear_velocity + self.angular_velocity)

        #Otherwise Normalize the speeds
        #Copied from/Inspired by WPILib Differential Drive normalization
        max_input = max(abs(self.linear_velocity),abs(self.angular_velocity))
        
        #Avoids div by 0 if 0 velocity is commanded
        if (max_input != 0):
            saturation_factor = (abs(self.linear_velocity) + abs(self.angular_velocity))/max_input
            left_norm = left_command/saturation_factor
            right_norm = right_command/saturation_factor
        
        else:
            left_norm = 0
            right_norm = 0


        #Scale the Drive speeds from unity accordingly
        left_velocity = RPS_FACTOR * GEAR_RATIO * left_norm
        right_velocity = RPS_FACTOR * GEAR_RATIO * right_norm

        return left_velocity, right_velocity


def main(args=None):
    rclpy.init(args=args)
    node = DriveCanControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
