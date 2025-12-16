import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import ControlMode, AxisState
import time

class TankDriveNode(Node):
    def __init__(self):
        super().__init__('tank_drive_node')

        self.get_logger().info('Searching for ODrive...')
        
        # Connect to ODrives (CAN or USB)
        self.odrv_0 = odrive.find_any(serial_number="0000-0000-0000-0001")  # Adjust serial number if needed
        self.odrv_1 = odrive.find_any(serial_number="0000-0000-0000-0002")  # Adjust serial number if needed
        self.odrv_2 = odrive.find_any(serial_number="0000-0000-0000-0003")  # Adjust serial number if needed
        self.odrv_3 = odrive.find_any(serial_number="0000-0000-0000-0004")  # Adjust serial number if needed
        self.odrv_4 = odrive.find_any(serial_number="0000-0000-0000-0005")  # Adjust serial number if needed
        self.odrv_5 = odrive.find_any(serial_number="0000-0000-0000-0006")  # Adjust serial number if needed
        
        self.get_logger().info('ODrives found.')

        # Setup axes for each motor
        self.left_axes = [
            self.odrv_0.axis0,  # Left motor 1 (ID 0)
            self.odrv_2.axis0,  # Left motor 2 (ID 2)
            self.odrv_4.axis0   # Left motor 3 (ID 4)
        ]
        self.right_axes = [
            self.odrv_1.axis0,  # Right motor 1 (ID 1)
            self.odrv_3.axis0,  # Right motor 2 (ID 3)
            self.odrv_5.axis0   # Right motor 3 (ID 5)
        ]

        # Setup all axes for velocity control
        for axis in self.left_axes + self.right_axes:
            axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

        # Define max velocity (in units per second, adjust as needed)
        self.max_velocity = 10  # You can adjust this to your system's max velocity

        # Subscribe to joystick input (left joystick controls left motors, right joystick controls right motors)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Assuming axis 1 and 4 for left and right joystick axes for velocity control
        left_joystick_value = msg.axes[1]  # Left joystick (forward/backward)
        right_joystick_value = msg.axes[4]  # Right joystick (forward/backward)

        # Scale joystick values to max velocity
        left_velocity = left_joystick_value * self.max_velocity
        right_velocity = right_joystick_value * self.max_velocity

        # Apply velocity commands to all motors on both sides
        for axis in self.left_axes:
            axis.controller.input_vel = left_velocity

        for axis in self.right_axes:
            axis.controller.input_vel = right_velocity


def main(args=None):
    rclpy.init(args=args)
    node = TankDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
