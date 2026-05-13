import serial
import time
import random
import struct

node = serial.Serial('COM37',500000, timeout=0.01)

test_packets = [
    [0x01, 0x03, 0x01, 0x00, 0x01, 0x10, 0x44, 0x6a],               #read first float register (should be pi)
    [0x01, 0x03, 0x03, 0x00, 0x03, 0x05, 0x85, 0x7d],
    [0x02, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x42, 0x42, 0x02, 0x31],     #write to slave ID 2 16962 int reg 1
    [0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18],
    [0x00],
    [0x01, 0x10, 0x03, 0x06, 0x03, 0x07, 0x01, 0x01, 0x69, 0xe0],
    [0x01, 0x03, 0x03, 0x06, 0x03, 0x06, 0x25, 0x7d]]


responces = []

hugePacket = []
for i in range(300):
    hugePacket.append(random.randrange(0,255,1))
test_packets[4] = hugePacket                         #adds 3000 bytes of noise to triger overflows, wrap arrounds, and worst case noise

for packet in test_packets:
    response_bytes = []
    print(bytes(packet))
    node.write(bytes(packet))
    time.sleep(0.05)
    for i in range(node.in_waiting):
        response_bytes.append(node.read())
    print(response_bytes)
    print("")
    responces.append(response_bytes)

"""
floatData = responces[0]
boolData = responces[1]

floatData = floatData[3:-2]

print("")
print(len(floatData))
print("")

print(floatData)

for i in range(0, len(floatData), 4):
    bin = b''.join(floatData[i:i+4])
    print(bin)
    flt = struct.unpack('f',bin)
    print(flt)
"""
