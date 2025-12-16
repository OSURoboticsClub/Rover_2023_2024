
import struct
import can
import cv2
import sys

can_net = sys.argv[1]
bus = can.interface.Bus(can_net, interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass


nodes = [0,1,2,3,4,5,6]
for node_id in nodes:
    print("Clearing {}".format(node_id))

    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x18), # 0x0d: Clear_errors
    data=struct.pack('<I', 1), 
    is_extended_id=False
    ))

    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # Set axis state
    data=struct.pack('<I', 8), # 8: closed loop control
    is_extended_id=False
    ))
