#!/usr/bin/env python

#####################################
# Imports
#####################################
import rclpy
from rclpy.node import Node
from time import time

import serial.rs485
import minimalmodbus

from rover2_control_interface.msg import DriveControlMessage, DriveStatusMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "scimech_control"

DEFAULT_PORT = "/dev/rover/ttyBogie"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_SCIMECH_CONTROL_MAIN_ACTUATOR_TOPIC = "scimech_control/main_actuator"
DEFAULT_SCIMECH_CONTROL_FLEXINOL_TOPIC = "scimech_control/flexinol"
DEFAULT_SCIMECH_CONTROL_SECONDARY_ACTUATOR_TOPIC = "scimech_control/secondary_actuator"
DEFAULT_DRIVE_CONTROL_STATUS_TOPIC = "drive_status/rear"

FIRST_MOTOR_ID = 1
SECOND_MOTOR_ID = 2
THIRD_MOTOR_ID = 3

COMMUNICATIONS_TIMEOUT = 0.01  # Seconds
RX_DELAY = 0.01
TX_DELAY = 0.01
DEFAULT_HERTZ = 30

MODBUS_REGISTERS = {
    "DIRECTION": 0,
    "SPEED": 1,
    "SLEEP": 2,
    "CURRENT": 3,
    "FAULT": 4,
    "TEMPERATURE": 5
}

MOTOR_DRIVER_DEFAULT_MESSAGE = [
    1,  # Forwards
    0,  # 0 Speed
    1   # Not in sleep mode
]

UINT16_MAX = 65535
BOGIE_LAST_SEEN_TIMEOUT = 2  # seconds

#####################################
# DriveControl Class Definition
#####################################
class DriveControl(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.port = self.declare_parameter("~port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("~baud", DEFAULT_BAUD).value

        self.first_motor_id = self.declare_parameter("~first_motor_id", FIRST_MOTOR_ID).value
        self.second_motor_id = self.declare_parameter("~second_motor_id", SECOND_MOTOR_ID).value
        self.third_motor_id = self.declare_parameter("~third_motor_id", THIRD_MOTOR_ID).value

        self.first_motor_inverted = self.declare_parameter("~first_motor_inverted", DEFAULT_INVERT).value
        self.second_motor_inverted = self.declare_parameter("~second_motor_inverted", DEFAULT_INVERT).value
        self.third_motor_inverted = self.declare_parameter("~third_motor_inverted", DEFAULT_INVERT).value

        self.scimech_control_main_actuator_topic = self.declare_parameter("~scimech_control_topic_main_actuator", DEFAULT_SCIMECH_CONTROL_MAIN_ACTUATOR_TOPIC).value
        self.scimech_control_flexinol_topic = self.declare_parameter("~scimech_control_topic_flexinol", DEFAULT_SCIMECH_CONTROL_FLEXINOL_TOPIC).value
        self.scimech_control_secondary_actuator_topic = self.declare_parameter("~~scimech_control_topic_secondary_actuator", DEFAULT_SCIMECH_CONTROL_SECONDARY_ACTUATOR_TOPIC).value
        self.drive_control_status_topic = self.declare_parameter("~drive_control_status_topic", DEFAULT_DRIVE_CONTROL_STATUS_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter("~hertz", DEFAULT_HERTZ).value

        self.first_motor = None
        self.second_motor = None
        self.third_motor = None

        self.connect_to_bogie()

        self.scimech_control_main_actuator_subscriber = self.create_subscription(
            DriveControlMessage,
            self.scimech_control_main_actuator_topic,
            self.main_actuator_callback,
            10
        )
        self.scimech_control_flexinol_subscriber = self.create_subscription(
            DriveControlMessage,
            self.scimech_control_flexinol_topic,
            self.flexinol_callback,
            10
        )
        self.scimech_control_secondary_actuator_subscriber = self.create_subscription(
            DriveControlMessage,
            self.scimech_control_secondary_actuator_topic,
            self.secondary_actuator_callback,
            10
        )

        self.drive_control_status_publisher = self.create_publisher(
            DriveStatusMessage,
            self.drive_control_status_topic,
            10
        )

        self.main_actuator_message = DriveControlMessage()
        self.flexinol_message = DriveControlMessage()
        self.secondary_actuator_message = DriveControlMessage()
        self.new_main_actuator_message = False
        self.new_flexinol_message = False
        self.new_secondary_actuator_message = False

        self.bogie_last_seen = time()
        self.timer = self.create_timer(self.wait_time, self.main_loop)

    def __setup_minimalmodbus_for_485(self):
        for motor in [self.first_motor, self.second_motor, self.third_motor]:
            motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
            motor.serial.rs485_mode = serial.rs485.RS485Settings(
                rts_level_for_rx=1, rts_level_for_tx=0,
                delay_before_rx=RX_DELAY,
                delay_before_tx=TX_DELAY
            )

    def connect_to_bogie(self):
        self.first_motor = minimalmodbus.Instrument(self.port, int(self.first_motor_id))
        self.second_motor = minimalmodbus.Instrument(self.port, int(self.second_motor_id))
        self.third_motor = minimalmodbus.Instrument(self.port, int(self.third_motor_id))
        self.__setup_minimalmodbus_for_485()

    def main_actuator_callback(self, drive_control):
        self.main_actuator_message = drive_control
        self.new_main_actuator_message = True
        
    def flexinol_callback(self, drive_control):
        self.flexinol_message = drive_control
        self.new_flexinol_message = True

    def secondary_actuator_callback(self, drive_control):
        self.secondary_actuator_message = drive_control
        self.new_secondary_actuator_message = True
        print("Secondary actuator!")
    def main_loop(self):
        try:
            self.send_main_actuator_message()
            self.send_flexinol_message()
            self.send_secondary_actuator_message()
   
            self.get_drive_status()
        except Exception as e:
            print(e)
            pass

        #if (time() - self.bogie_last_seen) > BOGIE_LAST_SEEN_TIMEOUT:
        #    print(f"Bogie not seen for {BOGIE_LAST_SEEN_TIMEOUT} seconds. Exiting.")
        #    self.destroy_node()

    def send_main_actuator_message(self):
        if not self.new_main_actuator_message:
            return 
        try:
            drive_control = self.main_actuator_message

            # third motor
            third_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
            third_data[MODBUS_REGISTERS["DIRECTION"]] = not drive_control.first_motor_direction if self.first_motor_inverted else drive_control.first_motor_direction
            third_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.first_motor_speed, UINT16_MAX)
            print(third_data)
            self.third_motor.write_registers(0, third_data)
            
        except Exception:
            pass

        self.new_main_actuator_message = False

    def send_flexinol_message(self):
        if not self.new_flexinol_message:
            return 

        try:
            drive_control = self.flexinol_message

            # second motor
            second_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
            second_data[MODBUS_REGISTERS["DIRECTION"]] = not drive_control.first_motor_direction if self.first_motor_inverted else drive_control.first_motor_direction
            second_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.first_motor_speed, UINT16_MAX)
            print(second_data)
            self.second_motor.write_registers(0, second_data)

        except Exception:
            pass

        self.new_flexinol_message = False


    def send_secondary_actuator_message(self): 
        if not self.new_secondary_actuator_message:
            return 
        try:
            drive_control = self.secondary_actuator_message
            # third motor
            first_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
            first_data[MODBUS_REGISTERS["DIRECTION"]] = not drive_control.first_motor_direction if self.first_motor_inverted else drive_control.first_motor_direction
            first_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.first_motor_speed, UINT16_MAX)
            print(first_data)
            self.first_motor.write_registers(0, first_data)
            
        except Exception as e:
            print(e)
            pass
        self.new_secondary_actuator_message = False


    def get_drive_status(self):
        status = DriveStatusMessage()

        try:
            first = self.first_motor.read_registers(3, 3)
            status.first_motor_connected = True
            status.first_motor_current = first[0] / 1000.0
            status.first_motor_fault = first[1] != 0
            status.first_motor_temp = first[2] / 1000.0
        except Exception:
            status.first_motor_connected = False

        try:
            second = self.second_motor.read_registers(3, 3)
            status.second_motor_connected = True
            status.second_motor_current = second[0] / 1000.0
            status.second_motor_fault = second[1] != 0
            status.second_motor_temp = second[2] / 1000.0
        except Exception:
            status.second_motor_connected = False

        try:
            third = self.third_motor.read_registers(3, 3)
            status.third_motor_connected = True
            status.third_motor_current = third[0] / 1000.0
            status.third_motor_fault = third[1] != 0
            status.third_motor_temp = third[2] / 1000.0
        except Exception:
            status.third_motor_connected = False

        if status.first_motor_connected or status.second_motor_connected or status.third_motor_connected:
            self.bogie_last_seen = time()

        self.drive_control_status_publisher.publish(status)

#####################################
# Main Entry Point
#####################################
def main(args=None):
    rclpy.init(args=args)
    drive_control = DriveControl()
    rclpy.spin(drive_control)
    drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
