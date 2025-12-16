#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rclpy
from rclpy.node import Node

from time import time, sleep

import serial.rs485
# import modbus_master as minimalmodbus
import minimalmodbus

from std_msgs.msg import UInt8, UInt16

# Custom Imports
from rover2_control_interface.msg import TowerPanTiltControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "chassis_pan_tilt_control"

DEFAULT_PORT = "/dev/rover/ttyChassisPanTilt"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_PAN_TILT_CONTROL_TOPIC = "chassis/pan_tilt/control"

PAN_TILT_NODE_ID = 2

COMMUNICATIONS_TIMEOUT = 0.01  # Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 60

PAN_TILT_MODBUS_REGISTERS = {
    "CENTER_ALL": 0,

    "PAN_ADJUST_POSITIVE": 1,
    "PAN_ADJUST_NEGATIVE": 2,
    "TILT_ADJUST_POSITIVE": 3,
    "TILT_ADJUST_NEGATIVE": 4
}

PAN_TILT_CONTROL_DEFAULT_MESSAGE = [
    0,  # No centering
    0,  # No pan positive adjustment
    0,  # No pan negative adjustment
    0,  # No tilt positive adjustment
    0  # No tilt negative adjustement
]

NODE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################

# class based on the Node object, which gives access to Node functions
# wary traveler, visit https://docs.ros2.org/foxy/api/rclpy/api/node.html for node basics
class ChassisPanTiltControl(Node):
    def __init__(self):
        # initializes the node, calling it what NODE_NAME stores
        super().__init__(NODE_NAME)

        # creates several variables within the Node object in question
        self.port = self.declare_parameter("~port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("~baud", DEFAULT_BAUD).value

        self.pan_tilt_node_id = self.declare_parameter("~pan_tilt_node_id", PAN_TILT_NODE_ID).value
        self.pan_tilt_control_subscriber_topic = self.declare_parameter("~pan_tilt_control_topic",
                                                                        DEFAULT_PAN_TILT_CONTROL_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter("~hertz", DEFAULT_HERTZ).value

        self.pan_tilt_node = None
        self.tower_node = None

        self.connect_to_pan_tilt_and_tower()

        self.pan_tilt_control_subscriber = self.create_subscription(TowerPanTiltControlMessage,
                                                                    self.pan_tilt_control_subscriber_topic,
                                                                    self.pan_tilt_control_callback, 1)

        self.pan_tilt_control_message = None
        self.new_pan_tilt_control_message = False

        self.modbus_nodes_seen_time = time()

        self.timer = self.create_timer(self.wait_time, self.main_loop)

    def __setup_minimalmodbus_for_485(self):
        self.pan_tilt_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.pan_tilt_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                          delay_before_rx=RX_DELAY,
                                                                          delay_before_tx=TX_DELAY)

    def main_loop(self):
        try:
            self.send_pan_tilt_control_message()
            self.modbus_nodes_seen_time = time()

        except Exception as error:
            # print(error)
            pass

        if (time() - self.modbus_nodes_seen_time) > NODE_LAST_SEEN_TIMEOUT:
            print(f"Chassis pan/tilt not seen for {NODE_LAST_SEEN_TIMEOUT} seconds. Exiting.")
            self.destroy_node()
            return  # Exit so respawn can take over

    def connect_to_pan_tilt_and_tower(self):
        self.pan_tilt_node = minimalmodbus.Instrument(self.port, int(self.pan_tilt_node_id))
        self.__setup_minimalmodbus_for_485()

    def send_startup_centering_command(self):
        try:
            registers = list(PAN_TILT_CONTROL_DEFAULT_MESSAGE)
            registers[PAN_TILT_MODBUS_REGISTERS["CENTER_ALL"]] = 1
            self.pan_tilt_node.write_registers(0, registers)
        except Exception as e:
            pass

    def send_pan_tilt_control_message(self):
        if self.new_pan_tilt_control_message:
            pan_tilt_control_message = self.pan_tilt_control_message  # type: TowerPanTiltControlMessage

            registers = list(PAN_TILT_CONTROL_DEFAULT_MESSAGE)
            registers[PAN_TILT_MODBUS_REGISTERS["CENTER_ALL"]] = int(pan_tilt_control_message.should_center)

            if pan_tilt_control_message.relative_pan_adjustment >= 0:
                registers[
                    PAN_TILT_MODBUS_REGISTERS["PAN_ADJUST_POSITIVE"]] = pan_tilt_control_message.relative_pan_adjustment
            else:
                registers[PAN_TILT_MODBUS_REGISTERS[
                    "PAN_ADJUST_NEGATIVE"]] = -pan_tilt_control_message.relative_pan_adjustment

            if pan_tilt_control_message.relative_tilt_adjustment >= 0:
                registers[PAN_TILT_MODBUS_REGISTERS[
                    "TILT_ADJUST_POSITIVE"]] = pan_tilt_control_message.relative_tilt_adjustment
            else:
                registers[PAN_TILT_MODBUS_REGISTERS[
                    "TILT_ADJUST_NEGATIVE"]] = -pan_tilt_control_message.relative_tilt_adjustment

            self.pan_tilt_node.write_registers(0, registers)

            self.new_pan_tilt_control_message = False
        else:
            self.pan_tilt_node.write_registers(0, PAN_TILT_CONTROL_DEFAULT_MESSAGE)

    def pan_tilt_control_callback(self, pan_tilt_control):
        self.pan_tilt_control_message = pan_tilt_control
        self.new_pan_tilt_control_message = True

# called when this file is ran (not as a script)
def main(args=None):
    # initializes rclpy with arguments
    rclpy.init(args=args)
    # assigns the variable cptc to a Node object class
    cptc = ChassisPanTiltControl()
    # runs the Node cptc
    rclpy.spin(cptc)
    cptc.destroy_node()
    rclpy.shutdown()

# called when this file is ran (as a script)
if __name__ == "__main__":
    main()
