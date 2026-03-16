import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import Trigger
from rover2_control_interface.msg import DriveCommandMessage
import time
import math


MAX_ACCEL = 0.05

RPS_FACTOR = 50/(2*math.pi) #GEAR RATIO / 2PI

class JoyToDriveNode(Node):
    def __init__(self):
        super().__init__('joy_to_drive')

        # Publisher Drive Commands
        self.wheel_pub = self.create_publisher(Twist, '/drive_controller/cmd_vel_unstamped', 1)

        # Drive Command Subscribers
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
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',  # Topic where joy messages are published
            self.joy_callback,
            1
        )

        self.start_drive_service = self.create_service(Trigger, "start_teleop_drive", self.start_teleop_cb)
        self.stop_drive_service = self.create_service(Trigger, "stop_teleop_drive", self.stop_teleop_cb)

        #Drive Control Loop
        self.timer = self.create_timer(0.03, self.timer_callback) 

        #Drive Parameters
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.max_vel = 1.1684  # Max linear velocity (m/s)
        self.max_ang_vel = 4.0  # Max angular velocity (rad/s)
        #Watchdog Timer
        self.last_message_time = time.time()
        self.publish_msgs = True

    def timer_callback(self):
        if self.publish_msgs:
            #Watchdog
            if time.time() >= self.last_message_time+2:    
                self.linear_velocity = 0.0  
                self.angular_velocity = 0.0  

            #Publish current velocity commands 
            twist = Twist()
            twist.linear.x = self.linear_velocity * self.max_vel
            twist.angular.z = self.angular_velocity * self.max_ang_vel 
            self.wheel_pub.publish(twist)

    def start_teleop_cb(self, reqest, response):
        self.publish_msgs = True
        response.success = True
        response.message = "Started Teleop Drive"
        return response
    
    def stop_teleop_cb(self, reqest, response):
        self.publish_msgs = False
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.wheel_pub.publish(twist)
        response.success = True
        response.message = "Started Teleop Drive"
        return response

    def joy_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        self.linear_velocity = msg.axes[1]   # Left joystick vertical axis (forward/backward)
        self.angular_velocity = msg.axes[3]  # Right joystick horizontal axis (turning)
        self.last_message_time = time.time()

    def groundstation_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
        self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)
        self.last_message_time = time.time()

    def iris_drive_command_callback(self, msg):
        # Map joystick axes to wheel velocities
        # Assume left stick y-axis for forward/backward and right stick x-axis for turning
        if msg.controller_present:
            self.linear_velocity = msg.drive_twist.linear.x  # Left joystick vertical axis (forward/backward)
            self.angular_velocity = msg.drive_twist.angular.z  # Right joystick horizontal axis (turning)
            self.last_message_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = JoyToDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

