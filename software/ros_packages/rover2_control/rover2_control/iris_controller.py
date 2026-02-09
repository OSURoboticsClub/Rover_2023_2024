#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rclpy
from rclpy.node import Node

from time import time,sleep
import serial.rs485
import minimalmodbus
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

# Custom Imports
from rover2_control_interface.msg import DriveCommandMessage, IrisStatusMessage
#####################################
# Global Variables
#####################################
NODE_NAME = "iris_controller"

DEFAULT_PORT = "/dev/rover/ttyIRIS"
DEFAULT_BAUD = 115200

DEFAULT_DRIVE_COMMAND_TOPIC = "command_control/iris_drive"
DEFAULT_IRIS_STATUS_TOPIC = "iris_status"
DEFAULT_JOY_COMMAND_TOPIC = "joy"
DEFAULT_JOY2_COMMAND_TOPIC = "joy2"
DEFAULT_ARM_POSE_COMMAND_TOPIC = "set_joint_angles"


DEFAULT_MOVEIT_CONTROLLER_JOY_BUTTON = 8


DEFAULT_HERTZ = 100
COMMUNICATIONS_TIMEOUT = 0.15  # Seconds

MODBUS_ID = 1
RPS_FACTOR = 4

STICK_DEADZONE = 0.01

RX_DELAY = 0.01
TX_DELAY = 0.01

SBUS_VALUES = {
    "SBUS_MAX": 1811,
    "SBUS_MID": 991,
    "SBUS_MIN": 172,
    "SBUS_RANGE": 820.0,
    "SBUS_DEADZONE": 5
}

MODBUS_REGISTERS = {
    "LEFT_STICK_Y_AXIS": 0,
    "RIGHT_STICK_Y_AXIS": 1,
    "RIGHT_STICK_X_AXIS": 2,
    "LEFT_STICK_X_AXIS": 3,
    "LEFT_POT": 4,
    "S1_POT": 5,
    "S2_POT": 6,
    "RIGHT_POT": 7,
    "SA_SWITCH": 8,
    "SB_SWITCH": 9,
    "SC_SWITCH": 10,
    "SD_SWITCH": 11,
    "SE_SWITCH": 12,
    "SF_SWITCH": 13,
    "SG_SWITCH": 14,
    "SH_SWITCH": 15,

    "VOLTAGE_24V": 16,
    "VOLTAGE_5V": 17,
    "USB_VOLTAGE_5V": 18,
    "VOLTAGE_3V3": 19
}

REGISTER_STATE_MAPPING = {
    "POSE_VS_IK_CONTROL": "SF_SWITCH",
    "DRIVE_VS_NEUTRAL_VS_ARM": "SE_SWITCH",
    "GROUND_VS_BOARD_POSE": "SA_SWITCH",
    "STRAIGHT_VS_STOWED_POSE": "SB_SWITCH",
}

ARM_POSES = {
    "PICKUP": [0.0, -0.698132, -1.65806, 0.0, -0.785398, 0.0],
    "ARM_BOARD": [0.0, -0.34, -1.98968, 0.0, 0.785398, 0.0],
    "STRAIGHT": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "STOWED": [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0],
}

IRIS_LAST_SEEN_TIMEOUT = 1  # seconds


#####################################
# IrisController Class Definition
#####################################
class IrisController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.port = self.declare_parameter('~port', DEFAULT_PORT).value
        self.baud = self.declare_parameter('~baud', DEFAULT_BAUD).value

        self.drive_command_publisher_topic = self.declare_parameter('~drive_command_topic', DEFAULT_DRIVE_COMMAND_TOPIC).value
        self.iris_status_publisher_topic = self.declare_parameter('~iris_status_topic', DEFAULT_IRIS_STATUS_TOPIC).value
        self.joy_command_publisher_topic = self.declare_parameter('~joy_command_topic', DEFAULT_JOY_COMMAND_TOPIC).value
        self.joy2_command_publisher_topic = self.declare_parameter('~joy2_command_topic', DEFAULT_JOY2_COMMAND_TOPIC).value
        self.arm_pose_command_publisher_topic = self.declare_parameter('~arm_pose_command_topic', DEFAULT_ARM_POSE_COMMAND_TOPIC).value

        self.wait_time = 1.0 / self.declare_parameter('~hertz', DEFAULT_HERTZ).value

        self.iris = minimalmodbus.Instrument(self.port, MODBUS_ID)
        self.__setup_minimalmodbus_for_485()

        self.drive_command_publisher = self.create_publisher(DriveCommandMessage, self.drive_command_publisher_topic, 1)
        self.iris_status_publisher = self.create_publisher(IrisStatusMessage, self.iris_status_publisher_topic, 1)
        self.joy_command_publisher = self.create_publisher(Joy,self.joy_command_publisher_topic, 1)
        self.joy2_command_publisher = self.create_publisher(Joy,self.joy2_command_publisher_topic, 1)
        self.arm_pose_command_publisher = self.create_publisher(Float32MultiArray,self.arm_pose_command_publisher_topic, 1)
       

        self.registers = []

        self.iris_connected = False

        self.iris_last_seen_time = time()

        self.timer = self.create_timer(self.wait_time, self.main_loop)

        self.published_pose_controller = False

        self.published_ik_controller = True

        
        self.sa_switch_toggle = False

        self.sb_switch_toggle = False
    def __setup_minimalmodbus_for_485(self):
        self.iris.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.iris.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                 delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

    def main_loop(self):
        try:

            self.read_registers()
            self.broadcast_drive_if_current_mode()
            self.broadcast_arm_if_current_mode()
            self.broadcast_iris_status()
            self.broadcast_arm_change_controller()
      
        except Exception as error:
            print(f"IRIS: Error occurred: {error}")

        if (time() - self.iris_last_seen_time) > IRIS_LAST_SEEN_TIMEOUT:
            print(f"Iris not seen for {IRIS_LAST_SEEN_TIMEOUT} seconds. Exiting.")
            self.destroy_node()
            return  # Exit so respawn can take over

    def read_registers(self):
        try:
            self.registers = self.iris.read_registers(0, len(MODBUS_REGISTERS))
            self.iris_last_seen_time = time()
        except Exception as error:
            self.iris_connected = False

    def broadcast_drive_if_current_mode(self):
        if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["DRIVE_VS_NEUTRAL_VS_ARM"]]] < SBUS_VALUES["SBUS_MID"] - SBUS_VALUES["SBUS_DEADZONE"]:
            command = DriveCommandMessage()
            #print("Drive command")
            left_y_axis = self.registers[MODBUS_REGISTERS["LEFT_STICK_Y_AXIS"]]
            right_x_axis = self.registers[MODBUS_REGISTERS["RIGHT_STICK_X_AXIS"]]
            
            if left_y_axis == 0 and right_x_axis == 0:
                command.controller_present = False
                command.ignore_drive_control = True
                command.drive_twist.linear.x = 0.0
                command.drive_twist.angular.z = 0.0
            
            else:

                left = (left_y_axis - SBUS_VALUES["SBUS_MID"]) / SBUS_VALUES[
                    "SBUS_RANGE"]

                right = (right_x_axis - SBUS_VALUES["SBUS_MID"]) / SBUS_VALUES[
                    "SBUS_RANGE"]

                command.controller_present = True
                command.drive_twist.linear.x = left
                command.drive_twist.angular.z = -right
                if abs(command.drive_twist.linear.x) < 0.025:
                    command.drive_twist.linear.x = 0.0
                if abs(command.drive_twist.angular.z) < 0.025:
                    command.drive_twist.angular.z = 0.0

            self.drive_command_publisher.publish(command)

    def broadcast_arm_if_current_mode(self):
        if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["DRIVE_VS_NEUTRAL_VS_ARM"]]] > \
                        SBUS_VALUES["SBUS_MID"] + SBUS_VALUES["SBUS_DEADZONE"]:
            #print("Arm")


           



            if self.published_pose_controller:
                arm_pose = []

                if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["GROUND_VS_BOARD_POSE"]]] < \
                    SBUS_VALUES["SBUS_MID"] + SBUS_VALUES["SBUS_DEADZONE"] and \
                  self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["GROUND_VS_BOARD_POSE"]]] > \
                    SBUS_VALUES["SBUS_MID"] - SBUS_VALUES["SBUS_DEADZONE"]:
                    self.sa_switch_toggle = True

                if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["STRAIGHT_VS_STOWED_POSE"]]] < \
                    SBUS_VALUES["SBUS_MID"] + SBUS_VALUES["SBUS_DEADZONE"] and \
                  self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["STRAIGHT_VS_STOWED_POSE"]]] > \
                    SBUS_VALUES["SBUS_MID"] - SBUS_VALUES["SBUS_DEADZONE"]:
                    self.sb_switch_toggle = True


               # print(self.registers[MODBUS_REGISTERS["SG_SWITCH"]])
                if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["GROUND_VS_BOARD_POSE"]]] > \
                    SBUS_VALUES["SBUS_MID"] + SBUS_VALUES["SBUS_DEADZONE"] and self.sa_switch_toggle:
                    arm_pose = ARM_POSES["PICKUP"]
                    self.sa_switch_toggle = False                

                elif self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["GROUND_VS_BOARD_POSE"]]] < \
                    SBUS_VALUES["SBUS_MID"] - SBUS_VALUES["SBUS_DEADZONE"] and self.sa_switch_toggle:
                    arm_pose = ARM_POSES["ARM_BOARD"]                
                    self.sa_switch_toggle = False                

                elif self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["STRAIGHT_VS_STOWED_POSE"]]] > \
                    SBUS_VALUES["SBUS_MID"] + SBUS_VALUES["SBUS_DEADZONE"] and self.sb_switch_toggle:
                    arm_pose = ARM_POSES["STRAIGHT"]                
                    self.sb_switch_toggle = False                

                elif self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["STRAIGHT_VS_STOWED_POSE"]]] < \
                    SBUS_VALUES["SBUS_MID"] - SBUS_VALUES["SBUS_DEADZONE"] and self.sb_switch_toggle:
                    arm_pose = ARM_POSES["STOWED"]
                    self.sb_switch_toggle = False                

                else:
                    #print("No input")
                    return    
                
                print(arm_pose)
                msg = Float32MultiArray()
                msg.data = arm_pose
                self.arm_pose_command_publisher.publish(msg)
                #PUBLISH TO TOPIC

            if self.published_ik_controller:
                left_y_axis = self.registers[MODBUS_REGISTERS["LEFT_STICK_Y_AXIS"]]
                right_y_axis = self.registers[MODBUS_REGISTERS["RIGHT_STICK_Y_AXIS"]]
                left_x_axis = self.registers[MODBUS_REGISTERS["LEFT_STICK_X_AXIS"]]
                right_x_axis = self.registers[MODBUS_REGISTERS["RIGHT_STICK_X_AXIS"]]
                axes = [left_x_axis,left_y_axis, right_y_axis if right_y_axis < SBUS_VALUES["SBUS_MID"]-SBUS_VALUES["SBUS_DEADZONE"] else 0.0, 0.0, 0.0, right_y_axis if right_y_axis > SBUS_VALUES["SBUS_MID"]+SBUS_VALUES["SBUS_DEADZONE"] else 0.0, 0.0, 0.0]
                buttons = [0,1 if right_x_axis > SBUS_VALUES["SBUS_MID"]+SBUS_VALUES["SBUS_DEADZONE"]+50 else 0,1 if right_x_axis < SBUS_VALUES["SBUS_MID"]-SBUS_VALUES["SBUS_DEADZONE"] else 0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                difference = SBUS_VALUES["SBUS_MID"]-SBUS_VALUES["SBUS_MIN"]
                for i in range(len(axes)):
                    if axes[i] != 0.0:
                        axes[i] = ((axes[i]-SBUS_VALUES["SBUS_MID"])/difference) * RPS_FACTOR
                        self.get_logger().info(f"Axis {i}: {axes[i]}")       
                        if abs(axes[i]) < STICK_DEADZONE:
                            axes[i] = 0.0
                if sum(axes) != 0.0 or buttons[1] == 1 or buttons[2] == 1:
                    axes[5] *= -1 #This is to make sure both behave in the (0,1) number range)
                    print(buttons)
                    self.publish_joy_msg(axes,buttons)
                #EMULATE JOY        

    def publish_joy_msg(self,axes,buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.joy_command_publisher.publish(msg)
    def publish_joy2_msg(self,axes,buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.joy2_command_publisher.publish(msg)
        

    def broadcast_arm_change_controller(self):
        buttons = [0 for _ in range(16)]
        buttons_default = [0]*16
        axes = [0.0] * 8
        buttons[DEFAULT_MOVEIT_CONTROLLER_JOY_BUTTON] = 1
       
        if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["POSE_VS_IK_CONTROL"]]] > SBUS_VALUES["SBUS_MID"] and not self.published_pose_controller:
            print("Change controller")
            self.publish_joy2_msg(axes,buttons)
            self.publish_joy2_msg(axes,buttons_default) # This is needed for the controller switcher, as it will only start to look for a switch if the button it is mapped to has changed between inputs
            sleep(0.5)
            self.published_pose_controller = True
            self.published_ik_controller = False            
        
        elif self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["POSE_VS_IK_CONTROL"]]] < SBUS_VALUES["SBUS_MID"] and not self.published_ik_controller:            
            print("Change controller ik")
            self.publish_joy2_msg(axes,buttons)
            self.publish_joy2_msg(axes,buttons_default) # This is needed for the controller switcher, as it will only start to look for a switch if the button it is mapped to has changed between inputs
            sleep(0.5)
            self.published_pose_controller = False
            self.published_ik_controller = True
        

    def broadcast_iris_status(self):
        status_message = IrisStatusMessage()
        status_message.iris_connected = True
        status_message.voltage_24v = self.registers[MODBUS_REGISTERS["VOLTAGE_24V"]]
        self.iris_status_publisher.publish(status_message)


def main(args=None):
    rclpy.init(args=args)
    iris = IrisController()
    rclpy.spin(iris)
    iris.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
