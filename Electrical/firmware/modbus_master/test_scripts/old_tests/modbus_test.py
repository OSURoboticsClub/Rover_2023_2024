import serial
import time
import random

"""
in c

UInt16 ModRTU_CRC(byte[] buf, int len)
{
  UInt16 crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (UInt16)buf[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}
"""

def modRTU_CRC(packet,len):
    crc = 0xffff
    for i in range(len):
        crc ^= packet[i]

        for j in range(8,0,-1):
            if((crc & 0x001) != 0):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    crcar = crc.to_bytes(2, 'little')
    return crcar

node = serial.Serial('COM5',115200, timeout=0.01)

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

test_packets = [
    [0x01, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x42, 0x42, 0x16, 0xc1],#, #write 16962 to int reg
    [0x01, 0x10, 0x01, 0x00, 0x00, 0x01, 0x04, 0x40, 0x48, 0xf5, 0xc3, 0x6c, 0xdb], #write 3.14 to float reg
    [0x01, 0x10, 0x02, 0x00, 0x00, 0x01, 0x01, 0x42, 0x41, 0x85], #write 'B' to char reg
    [0x01, 0x10, 0x03, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0xa5], #write True to bool reg
    [0x69, 0x02],        #little bit of garbage data  This gets replaced with huge random packet
    [0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xf6],               #read first float register (should be pi)
    [0x00, 0x01, 0x02, 0x03, 0x90, 0x91, 0x92, 0x93, 0x94, 0x69],   #more garbage data
    [0x02, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x42, 0x42, 0x02, 0x31],     #write to slave ID 2 16962 int reg 1
    [0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x18],                        #responce from slave ID 2
    [0x01, 0x03, 0x00, 0x00, 0x00, 0x30, 0x45, 0xde]]                        #read from registers that have not been setup

hugePacket = []
for i in range(3000):
    hugePacket.append(random.randrange(0,255,1))
test_packets[4] = hugePacket                         #adds 3000 bytes of noise to triger overflows, wrap arrounds, and worst case noise

print("Running profile")
startTime = time.perf_counter()


for packet in test_packets:
    response_bytes = []
    print(bytes(packet))
    node.write(bytes(packet))
    time.sleep(0.05)
    for i in range(node.in_waiting):
        response_bytes.append(node.read())
    print(response_bytes)
    print("")
