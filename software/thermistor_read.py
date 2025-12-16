#!/usr/bin/python3

import struct
import can
import cv2
import matplotlib.pyplot as plt
from rich.live import Live
from rich.table import Table
import math
import sys

odrive_errors_by_code = {
    "0": "NO ERROR",
    "1": "INITIALIZING",
    "2": "SYSTEM_LEVEL",
    "4": "TIMING_ERROR",
    "8": "MISSING_ESTIMATE",
    "16": "BAD_CONFIG",
    "32": "DRV_FAULT",
    "64": "MISSING_INPUT",
    "256": "DC_BUS_OVER_VOLTAGE",
    "512": "DC_BUS_UNDER_VOLTAGE",
    "1024": "DC_BUS_OVER_CURRENT",
    "2048": "DC_BUS_OVER_REGEN_CURRENT",
    "4096": "CURRENT_LIMIT_VIOLATION",
    "8192": "MOTOR_OVER_TEMP",
    "16384": "INVERTER_OVER_TEMP",
    "32768": "VELOCITY_LIMIT_VIOLATION",
    "65536": "POSITION_LIMIT_VIOLATION",
    "16777216": "WATCHDOG_TIMER_EXPIRED",
    "33554432": "ESTOP_REQUESTED",
    "67108864": "SPINOUT_DETECTED",
    "134217728": "BRAKE_RESISTOR_DISARMED",
    "268435456": "THERMISTOR_DISCONNECTED",
    "1073741824": "CALIBRATION_ERROR"
}



nodes = [0,1,2,3,4,5,6]
node_fet_temp = [0,0,0,0,0,0,0]
node_motor_temp =  [0,0,0,0,0,0,0]
node_error_msg =[(0,0) for i in range(len(nodes)+1)]
node_velocity =  [0,0,0,0,0,0,0]
node_position =   [0,0,0,0,0,0,0]
can_network = sys.argv[1]
bus = can.interface.Bus(can_network, interface="socketcan")
node_iq_setpoint =   [0,0,0,0,0,0,0]
node_iq_measured =   [0,0,0,0,0,0,0]
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass


def generate_table():
    table = Table(title="odrive table")
    table.add_column("ID")
    table.add_column("FET Temp")
    table.add_column("MOTOR Temp")
    table.add_column("ACTIVE Error")
    table.add_column("DISARM Reason")
    table.add_column("VELOCITY")
    table.add_column("POSITION")
    table.add_column("IQ Setpoint")
    table.add_column("IQ Measured")

    for i in nodes:
#        print(node_motor_temp[i])
        table.add_row(str(i),str(node_fet_temp[i]),str(node_motor_temp[i]),str(node_error_msg[i][0]),odrive_errors_by_code[str(node_error_msg[i][1])],str(node_velocity[i]),str(node_position[i]),str(node_iq_setpoint[i]),str(node_iq_measured[i]))
    return table

def round(value, sigfig):
    value = value*(10**sigfig)
    if not math.isnan(value):
        value = int(value)
    value = value/(10**sigfig)
    return value

with Live(generate_table(), refresh_per_second=1) as live:
    for msg in bus:
        for node_id in nodes:
            if msg.arbitration_id == (node_id << 5 | 0x15): # 0x01: Heartbeat
                fetTemp, motorTemp = struct.unpack('<ff', bytes(msg.data))
#                print("Node ID: {}\tFet temp: {}\tMotor temp: {}".format(node_id,round(fetTemp,2), round(motorTemp,2)))
                node_fet_temp[node_id] = round(fetTemp,1)
                node_motor_temp[node_id] = round(motorTemp,1)
            
#                    print("WARNING WARNING WARNING WARNING WARNING")
#                    print("Node {} is over 80C".format(node_id)) 
#                    print("WARNING WARNING WARNING WARNING WARNING")

            if msg.arbitration_id == (node_id << 5 | 0x03): # Error   
            
                activeError, disarmReason = struct.unpack('<II',bytes(msg.data))
                node_error_msg[node_id] = (activeError, disarmReason)
            if msg.arbitration_id == (node_id << 5 | 0x09): # Error   
                pos_estimate,vel_estimate = struct.unpack('<ff',bytes(msg.data))
                node_position[node_id] = round(pos_estimate,3)
                node_velocity[node_id] = round(vel_estimate,3)
            if msg.arbitration_id == (node_id << 5 | 0x14): # Error   
                iq_setpoint,iq_measured = struct.unpack('<ff',bytes(msg.data))
                node_iq_setpoint[node_id] = round(iq_setpoint,1)
                node_iq_measured[node_id] = round(iq_measured,1)


        live.update(generate_table())
