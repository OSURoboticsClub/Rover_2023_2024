#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import serial
import time
import struct


class RTCMSerialNode(Node):
    def __init__(self):
        super().__init__('rtcm_serial_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 38400)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Publisher
        self.rtcm_pub = self.create_publisher(
            UInt8MultiArray,
            'rtcm',
            10
        )

        # Serial port
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.get_logger().info(f'Opened serial port {port} @ {baudrate} baud')

        self.buffer = bytearray()

        # Timer instead of while True
        self.timer = self.create_timer(0.01, self.read_serial)

    @staticmethod
    def is_rtcm(buf: bytearray) -> bool:
        # RTCM3 preamble
        return len(buf) >= 2 and buf[0] == 0xD3

    def parse_rtcm(self, data: bytes):
        msg = UInt8MultiArray()
        msg.data = list(data)
        self.rtcm_pub.publish(msg)
        self.get_logger().debug(f'Published RTCM3 message ({len(data)} bytes)')

    def read_serial(self):
        if not self.ser.in_waiting:
            return

        chunk = self.ser.read(self.ser.in_waiting)
        self.buffer.extend(chunk)
        print(self.buffer)
        print("")
        while self.buffer:
            # RTCM
            if len(self.buffer) >= 3 and self.is_rtcm(self.buffer):
                msg_len = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
                total_len = 3 + msg_len + 3

                if len(self.buffer) >= total_len:
                    msg = bytes(self.buffer[:total_len])
                    self.parse_rtcm(msg)
                    self.buffer = self.buffer[total_len:]
                    continue
                break

            else:
                self.buffer = self.buffer[1:]


def main():
    rclpy.init()
    node = RTCMSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
