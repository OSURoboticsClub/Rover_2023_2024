#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rclpy
from rclpy.node import Node

from time import time, sleep

# Custom Imports
from rover2_control_interface.msg import DriveCommandMessage, DriveControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "drive_coordinator"

DEFAULT_IRIS_DRIVE_COMMAND_TOPIC = "command_control/iris_drive"
DEFAULT_GROUND_STATION_DRIVE_COMMAND_TOPIC = "command_control/ground_station_drive"
DEFAULT_REAR_BOGIE_TOPIC = "drive_control/rear"
DEFAULT_LEFT_BOGIE_TOPIC = "drive_control/left"
DEFAULT_RIGHT_BOGIE_TOPIC = "drive_control/right"

UINT16_MAX = 65535

DEFAULT_HERTZ = 30

WATCHDOG_TIMEOUT = 0.3


#####################################
# ControlCoordinator Class Definition
#####################################
class DriveCoordinator(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.iris_drive_command_topic = self.declare_parameter("~iris_drive_command_topic", DEFAULT_IRIS_DRIVE_COMMAND_TOPIC).value
        self.ground_station_drive_command_topic = \
            self.declare_parameter("~ground_station_drive_command_topic", DEFAULT_GROUND_STATION_DRIVE_COMMAND_TOPIC).value
        self.rear_bogie_topic = self.declare_parameter("~rear_bogie_control_topic", DEFAULT_REAR_BOGIE_TOPIC).value
        self.left_bogie_topic = self.declare_parameter("~left_bogie_control_topic", DEFAULT_LEFT_BOGIE_TOPIC).value
        self.right_bogie_topic = self.declare_parameter("~right_bogie_control_topic", DEFAULT_RIGHT_BOGIE_TOPIC).value
        self.wait_time = 1.0 / self.declare_parameter("~hertz", DEFAULT_HERTZ).value

        # Drive data
        self.drive_command_data = {
            "iris": {
                "message": DriveCommandMessage(),
                "last_time": time()
            },

            "ground_station": {
                "message": DriveCommandMessage(),
                "last_time": time()
            }
        }
        print(DriveCommandMessage())
        # ########## Class Variables ##########
        self.iris_drive_command_subscriber = self.create_subscription(DriveCommandMessage,
                                                                self.iris_drive_command_topic,
                                                                self.iris_drive_command_callback,
                                                                1)
        self.ground_station_command_subscriber = self.create_subscription(DriveCommandMessage,
                                                                self.ground_station_drive_command_topic,
                                                                self.ground_station_drive_command_callback,
                                                                1)
                                                                
        self.rear_bogie_publisher = self.create_publisher(DriveControlMessage, self.rear_bogie_topic, 1)
        self.left_bogie_publisher = self.create_publisher(DriveControlMessage, self.left_bogie_topic, 1)
        self.right_bogie_publisher = self.create_publisher(DriveControlMessage, self.right_bogie_topic, 1)

        self.last_message_time = time()

        # ########## Run the Class ##########
        self.timer = self.create_timer(self.wait_time, self.main_loop)


    def process_drive_commands(self):
        if self.drive_command_data["ground_station"]["message"].controller_present:
            print("ground_station")
            print(self.drive_command_data["ground_station"]["message"])
            self.send_drive_control_command(self.drive_command_data["ground_station"])
        else:
            print("iris")
            self.send_drive_control_command(self.drive_command_data["iris"])

    def send_drive_control_command(self, drive_command_data):
        if (time() - drive_command_data["last_time"]) > WATCHDOG_TIMEOUT:
            drive_command = DriveCommandMessage()
        else:
            drive_command = drive_command_data["message"]

        rear_drive = DriveControlMessage()
        left_drive = DriveControlMessage()
        right_drive = DriveControlMessage()

        left = drive_command.drive_twist.linear.x - drive_command.drive_twist.angular.z
        right = drive_command.drive_twist.linear.x + drive_command.drive_twist.angular.z

        left_direction = left >= 0
        rear_drive.first_motor_direction = left_direction
        left_drive.first_motor_direction = left_direction
        left_drive.second_motor_direction = left_direction

        right_direction = right >= 0
        rear_drive.second_motor_direction = right_direction
        right_drive.first_motor_direction = right_direction
        right_drive.second_motor_direction = right_direction

        left_speed = min(int(abs((left if abs(left) > 0.03 else 0) * UINT16_MAX)), UINT16_MAX)
        right_speed = min(int(abs((right if abs(right) > 0.03 else 0) * UINT16_MAX)), UINT16_MAX)

        rear_drive.first_motor_speed = left_speed
        left_drive.first_motor_speed = left_speed
        left_drive.second_motor_speed = left_speed

        rear_drive.second_motor_speed = right_speed
        right_drive.first_motor_speed = right_speed
        right_drive.second_motor_speed = right_speed

        self.rear_bogie_publisher.publish(rear_drive)
        self.left_bogie_publisher.publish(left_drive)
        self.right_bogie_publisher.publish(right_drive)


    def iris_drive_command_callback(self, drive_command):
        self.drive_command_data["iris"]["message"] = drive_command
        self.drive_command_data["iris"]["last_time"] = time()

    def ground_station_drive_command_callback(self, drive_command):
        self.drive_command_data["ground_station"]["message"] = drive_command
        self.drive_command_data["ground_station"]["last_time"] = time()

def main(args=None):
    rclpy.init(args=args)
    drive_Cord = DriveCoordinator()
    rclpy.spin(drive_Cord)
    drive_Cord.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
