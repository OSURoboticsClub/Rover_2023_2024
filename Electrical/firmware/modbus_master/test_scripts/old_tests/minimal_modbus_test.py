import sys
sys.path.append('../../ros_packages/rover_control/src/minimalmodbus')

import minimalmodbus

"""
write command (from master)
[slave id, function code, start register bytes (x2), number of registers bytes (x2), number of bytes, data bytes (x???), CRC bytes(x2)]

write response (from slave)
[slave id, function code, start register bytes(x2), number of registers bytes (x2), CRC bytes (x2)]

read command (from master)
[slave id, function code, start register bytes(x2), number of registers bytes (x2), CRC bytes (x2)]

read response (from slave)
[slave id, function code, number of bytes, data bytes (x???), CRC bytes(x2)]
"""

node = minimalmodbus.Instrument('COM5', 1)
print(node.write_registers(5, [1, 2, 3, 4]))
print(node.read_registers(5, 4))
