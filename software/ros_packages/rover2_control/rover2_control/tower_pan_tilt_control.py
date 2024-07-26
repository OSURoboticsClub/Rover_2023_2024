#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rclpy
from rclpy.node import Node 
from time import time, sleep

import serial.rs485
import minimalmodbus

from std_msgs.msg import UInt8, UInt16

# Custom Imports
from rover2_control_interface.msg import TowerPanTiltControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "tower_pan_tilt_control"

DEFAULT_PORT = "/dev/rover/ttyTowerAndPanTilt"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_TOWER_LIGHT_CONTROL_TOPIC = "tower/light/control"
DEFAULT_TOWER_CO2_STATUS_TOPIC = "tower/status/co2"
DEFAULT_PAN_TILT_CONTROL_TOPIC = "tower/pan_tilt/control"

TOWER_NODE_ID = 1
PAN_TILT_NODE_ID = 2

COMMUNICATIONS_TIMEOUT = 0.02  # Seconds

RX_DELAY = 0.02
TX_DELAY = 0.02

DEFAULT_HERTZ = 20

PAN_TILT_MODBUS_REGISTERS = {
    "CENTER_ALL": 0,

    "PAN_ADJUST_POSITIVE": 1,
    "PAN_ADJUST_NEGATIVE": 2,
    "TILT_ADJUST_POSITIVE": 3,
    "TILT_ADJUST_NEGATIVE": 4,
    "HITCH_SERVO_POSITIVE": 5,
    "HITCH_SERVO_NEGATIVE": 6
}

TOWER_MODBUS_REGISTERS = {
    "LED_CONTROL": 0,
    "CO2_READING_PPM": 1
}

PAN_TILT_CONTROL_DEFAULT_MESSAGE = [
    0,  # No centering
    0,  # No pan positive adjustment
    0,  # No pan negative adjustment
    0,  # No tilt positive adjustment
    0,  # No tilt negative adjustement
    0,  # Hitch servo (+ dir) set to false
    0   # Hitch servo (- dir) set to false
]

TOWER_LIGHT_STATES = {
    "NO_CHANGE": 0,
    "LIGHT_OFF": 1,
    "LIGHT_FLASH": 2,
    "LIGHT_MED": 3,
    "LIGHT_HIGH": 4
}

TOWER_CONTROL_DEFAULT_MESSAGE = [
    TOWER_LIGHT_STATES["LIGHT_OFF"]  # Light off
]

NODE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################
class TowerPanTiltControl(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        
        self.port = self.declare_parameter("~port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("~baud", DEFAULT_BAUD).value

        self.tower_node_id = self.declare_parameter("~tower_node_id", TOWER_NODE_ID).value
        self.pan_tilt_node_id = self.declare_parameter("~pan_tilt_node_id", PAN_TILT_NODE_ID).value

        self.tower_light_control_subscriber_topic = self.declare_parameter("~tower_light_control_topic",
                                                                    DEFAULT_TOWER_LIGHT_CONTROL_TOPIC).value
        self.pan_tilt_control_subscriber_topic = self.declare_parameter("~pan_tilt_control_topic",
                                                                 DEFAULT_PAN_TILT_CONTROL_TOPIC).value

        self.tower_co2_publisher_topic = self.declare_parameter("~tower_co2_status_topic",
                                                         DEFAULT_TOWER_CO2_STATUS_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter("~hertz", DEFAULT_HERTZ).value

        self.pan_tilt_node = None
        self.tower_node = None

        self.connect_to_pan_tilt_and_tower()

        self.pan_tilt_control_subscriber = self.create_subscription(TowerPanTiltControlMessage, 
                                                                self.pan_tilt_control_subscriber_topic,
                                                                self.pan_tilt_control_callback, 1)

        self.tower_light_control_subscriber = self.create_subscription(UInt8, self.tower_light_control_subscriber_topic,
                                                                self.tower_light_control_callback, 1)

        self.tower_co2_publisher = self.create_publisher(UInt16, self.tower_co2_publisher_topic, 1)

        self.pan_tilt_control_message = None
        self.tower_light_control_message = None

        self.new_pan_tilt_control_message = False
        self.new_tower_light_control_message = False

        self.modbus_nodes_seen_time = time()

        self.send_startup_centering_and_lights_off_command()

        self.timer = self.create_timer(self.wait_time, self.main_loop)

    def __setup_minimalmodbus_for_485(self):
        self.pan_tilt_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.pan_tilt_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        self.tower_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.tower_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

    def main_loop(self):
        try:
            self.send_pan_tilt_control_message()
            self.modbus_nodes_seen_time = time()

        except Exception as error:
            pass
            # print("pantilt", error)
            # print "Error occurred:", error

        try:
            self.send_tower_control_message()
            self.broadcast_co2_reading_message()
            self.modbus_nodes_seen_time = time()

        except Exception as error:
            pass
            # print("tower", error)
            # print "Error occurred:", error

        if (time() - self.modbus_nodes_seen_time) > NODE_LAST_SEEN_TIMEOUT:
            print(f"Tower pan/tilt not seen for {NODE_LAST_SEEN_TIMEOUT} seconds. Exiting.")
            self.destroy_node()
            return  # Exit so respawn can take over

    def connect_to_pan_tilt_and_tower(self):
        self.tower_node = minimalmodbus.Instrument(self.port, int(self.tower_node_id))
        self.pan_tilt_node = minimalmodbus.Instrument(self.port, int(self.pan_tilt_node_id))
        self.__setup_minimalmodbus_for_485()

    def send_startup_centering_and_lights_off_command(self):
        try:
            # registers = list(PAN_TILT_CONTROL_DEFAULT_MESSAGE)
            # registers[PAN_TILT_MODBUS_REGISTERS["CENTER_ALL"]] = 1
            # self.pan_tilt_node.write_registers(0, registers)

            self.tower_node.write_register(0, TOWER_LIGHT_STATES["LIGHT_OFF"])
        except Exception as e:
            pass

    def send_pan_tilt_control_message(self):
        if self.new_pan_tilt_control_message:
            pan_tilt_control_message = self.pan_tilt_control_message  # type: TowerPanTiltControlMessage

            registers = list(PAN_TILT_CONTROL_DEFAULT_MESSAGE)
            registers[PAN_TILT_MODBUS_REGISTERS["CENTER_ALL"]] = int(pan_tilt_control_message.should_center)

            if pan_tilt_control_message.relative_pan_adjustment >= 0:
                registers[PAN_TILT_MODBUS_REGISTERS["PAN_ADJUST_POSITIVE"]] = pan_tilt_control_message.relative_pan_adjustment
            else:
                registers[PAN_TILT_MODBUS_REGISTERS["PAN_ADJUST_NEGATIVE"]] = -pan_tilt_control_message.relative_pan_adjustment

            if pan_tilt_control_message.relative_tilt_adjustment >= 0:
                registers[PAN_TILT_MODBUS_REGISTERS["TILT_ADJUST_POSITIVE"]] = pan_tilt_control_message.relative_tilt_adjustment
            else:
                registers[PAN_TILT_MODBUS_REGISTERS["TILT_ADJUST_NEGATIVE"]] = -pan_tilt_control_message.relative_tilt_adjustment
            if pan_tilt_control_message.hitch_servo_positive:
                registers[PAN_TILT_MODBUS_REGISTERS["HITCH_SERVO_POSITIVE"]] = 1
            elif pan_tilt_control_message.hitch_servo_negative:
                registers[PAN_TILT_MODBUS_REGISTERS["HITCH_SERVO_NEGATIVE"]] = 1

            self.pan_tilt_node.write_registers(0, registers)

            self.new_pan_tilt_control_message = False
        else:
            self.pan_tilt_node.write_registers(0, PAN_TILT_CONTROL_DEFAULT_MESSAGE)

    def broadcast_co2_reading_message(self):
        self.tower_co2_publisher.publish(UInt16(data=self.tower_node.read_register(1)))

    def send_tower_control_message(self):
        if self.new_tower_light_control_message:
            self.tower_node.write_register(0, self.tower_light_control_message.data)
            self.new_tower_light_control_message = False

    def pan_tilt_control_callback(self, pan_tilt_control):
        self.pan_tilt_control_message = pan_tilt_control
        self.new_pan_tilt_control_message = True

    def tower_light_control_callback(self, light_control):
        self.tower_light_control_message = light_control
        self.new_tower_light_control_message = True

def main(args=None):
    rclpy.init(args=args)
    tower_control = TowerPanTiltControl()
    rclpy.spin(tower_control)
    tower_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    TowerPanTiltControl()
