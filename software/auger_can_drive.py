import can
import struct
import sys
import time  # Add this import
import math

bus = can.interface.Bus("can1", interface="socketcan")
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass
#Get input for test drive
#vel = sys.argv[1]
#vel = float(vel)

RPS_FACTOR = 21
GEAR_RATIO = 55
GOAL_RPS = float(sys.argv[1])
VELOCITY = -1.0*GEAR_RATIO*GOAL_RPS
VEL_RAMP = GEAR_RATIO * RPS_FACTOR

node_id = 6

print("Setting {} to vel control".format(node_id))
# Put axis into closed loop control state
bus.send(can.Message(
arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
is_extended_id=False
))

#Set velocity ramp control mode
bus.send(can.Message(
arbitration_id=(node_id << 5 | 0x0b), 
data=struct.pack('<II', 2,2),
is_extended_id=False
))
#Set veloicity ramp speed
bus.send(can.Message(
arbitration_id=(node_id << 5 | 0x04), 
data=struct.pack('<BHBf', 1,403,0,VEL_RAMP),
is_extended_id=False
))

bus.send(can.Message(
arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
data=struct.pack('<ff', VELOCITY, 0.0), # 1.0: velocity, 0.0: torque feedforward
is_extended_id=False
))
