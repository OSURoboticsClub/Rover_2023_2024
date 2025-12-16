import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage
import time
import math


MAX_ACCEL = 0.05

RPS_FACTOR = 50/(2*math.pi) #GEAR RATIO / 2PI




class JoyToDriveNode(Node):
    def __init__(self):
        super().__init__('joy_to_drive')

        self.linear_velocity = 0
        self.angular_velocity = 0
        self.prev_linear_velocity = 0
        self.prev_angular_velocity = 0
        # Publisher for left and right wheel velocities
        #----
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/can_controller/commands', 1)
        #self.right_wheel_pub = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10.0)
        #----
        # Subscriber to joy topic

        self.groundstation_sub = self.create_subscription(
            DriveCommandMessage,
            '/command_control/ground_station_drive',  # Topic where joy messages are published
            self.groundstation_drive_command_callback,
            1
        )
        self.iris_sub = self.create_subscription(
            DriveCommandMessage,
            '/command_control/iris_drive',  # Topic where joy messages are published
            self.iris_drive_command_callback,
            1
        )

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x)) - 0.5
        
    def normalize_drive_commands(self):
    
        left_velocity = self.sigmoid(self.linear_velocity)
        right_velocity = self.sigmoid(self.angular_velocity)        

        left_velocity = -1 * (self.linear_velocity - self.angular_velocity)*RPS_FACTOR*2
        right_velocity = (self.linear_velocity + self.angular_velocity)*RPS_FACTOR*2
        drive_msg = Float64MultiArray()
        drive_msg.data = [right_velocity,right_velocity,right_velocity,left_velocity,left_velocity,left_velocity]

        self.prev_linear_velocity = left_velocity
        self.prev_angular_velocity = right_velocity
        
        # Publish the velocities
        self.wheel_pub.publish(drive_msg)

    def groundstation_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
        self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)
        
        # You may want to scale these velocities to suit your needs (e.g., make them faster/slower)
        self.normalize_drive_commands()

    def iris_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        if msg.controller_present:
            self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)
            # You may want to scale these velocities to suit your needs (e.g., make them faster/slower)
            self.normalize_drive_commands()

def main(args=None):
    rclpy.init(args=args)
    node = JoyToDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

