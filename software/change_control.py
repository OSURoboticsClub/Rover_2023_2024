import can
import struct
import sys
import time  # Add this import
import math

bus = can.interface.Bus("can0", interface="socketcan")
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass
#Get input for test drive
#vel = sys.argv[1]
#vel = float(vel)
RPS_FACTOR = 1
GEAR_RATIO = 50
vel_ramp = GEAR_RATIO * RPS_FACTOR

nodes = [0,1,2,3,4,5]
for node_id in nodes:
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
    #Set PI controls
    #DO NOT UNCOMMENT, HERE FOR REFERENCE BUT MOTORS MAKE GROSS NOISE
#    bus.send(can.Message(
#    arbitration_id=(node_id << 5 | 0x1b),  # 0x1b: Set_Vel_Gains
#    data=struct.pack('<ff', 0.2, 0.01),    # Example values: vel_gain=0.2, vel_integrator_gain=0.01
#    is_extended_id=False
#    ))
    #Set velocity ramp rate
    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), 
    data=struct.pack('<BHBf', 1,403,0,vel_ramp),
    is_extended_id=False
    ))
    
    #Test to see if ramp rate is being written correctly
    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB', 0, 403, 0),
    is_extended_id=False
    ))
    # Await reply
    for msg in bus:
        if msg.is_rx and msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
            break
    # Unpack and print reply
    _,_,_, return_value = struct.unpack_from('<BHBf', msg.data)
    print(f"received: {return_value}")
    
#for node_id in nodes:
#    bus.send(can.Message(
#    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
#    data=struct.pack('<ff', vel, 0.0), # 1.0: velocity, 0.0: torque feedforward
#    is_extended_id=False
#    ))
