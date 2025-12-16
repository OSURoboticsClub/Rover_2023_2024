import can
from dataclasses import dataclass
import json
import struct


bus = can.interface.Bus("can0", interface="socketcan")
# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass
nodes = [0,1,2,3,4,5]
REBOOT_CMD = 0x16

def reboot_msg(node_id, action: int):
        bus.send(can.Message(
            arbitration_id=(node_id << 5) | REBOOT_CMD,
            data=[action],
            is_extended_id=False
        ))

for node_id in nodes:
    print(node_id)
    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHBf', 0, 398, 0, 0.04),
    is_extended_id=False
    ))
    #Set velocity ramp control mode
    bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHBf', 0, 399, 0, 0.01),
    is_extended_id=False
    ))
    reboot_msg(node_id,1)