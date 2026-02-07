#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus

import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.ser = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
        self.get_logger().info(f"Serial port {self.ser.port} opened at {self.ser.baudrate} baud")

        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

        self.rtcm_sub = self.create_subscription(UInt8MultiArray, 'rtcm', self.rtcm_callback, 10)

        self.buffer = bytearray()
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                self.buffer.extend(chunk)

            while self.buffer:
                if self.buffer[0] == ord('$'):
                    newline_idx = self.buffer.find(b'\n')
                    if newline_idx != -1:
                        line = self.buffer[:newline_idx].decode('ascii', errors='ignore').strip()
                        self.parse_nmea(line)
                        self.buffer = self.buffer[newline_idx + 1:]
                        continue
                    break
                else:
                    # Discard unknown byte
                    self.buffer = self.buffer[1:]

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

    def parse_nmea(self, line):
        parts = line.split(',')
        if parts[0] in ['$GNGGA', '$GPGGA']:
            try:
                if len(parts) > 9 and parts[2] and parts[4]:
                    # Latitude
                    lat_raw = parts[2]
                    lat_deg = int(float(lat_raw) / 100)
                    lat_min = float(lat_raw) - (lat_deg * 100)
                    lat = lat_deg + (lat_min / 60.0)
                    if parts[3] == 'S':
                        lat = -lat

                    # Longitude
                    lon_raw = parts[4]
                    lon_deg = int(float(lon_raw) / 100)
                    lon_min = float(lon_raw) - (lon_deg * 100)
                    lon = lon_deg + (lon_min / 60.0)
                    if parts[5] == 'W':
                        lon = -lon

                    # Fix quality
                    fix_quality = int(parts[6])
                    # Number of satellites
                    num_sats = int(parts[7])
                    # Altitude
                    altitude = float(parts[9]) if parts[9] else 0.0

                    # Create NavSatFix message
                    fix_msg = NavSatFix()
                    fix_msg.header.stamp = self.get_clock().now().to_msg()
                    fix_msg.header.frame_id = 'gps'

                    fix_msg.status.status = NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
                    fix_msg.status.service = NavSatStatus.SERVICE_GPS

                    fix_msg.latitude = lat
                    fix_msg.longitude = lon
                    fix_msg.altitude = altitude
                    fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                    self.fix_pub.publish(fix_msg)
                    self.get_logger().debug(f"Published GPS fix: lat={lat}, lon={lon}, alt={altitude}, sats={num_sats}")

            except (ValueError, IndexError):
                self.get_logger().warn(f"Failed to parse NMEA line: {line}")

    def rtcm_callback(self, msg: UInt8MultiArray):
        """Write incoming RTCM bytes to GPS"""
        try:
            data = bytes(msg.data)
            self.ser.write(data)
            self.get_logger().debug(f"Wrote {len(data)} RTCM bytes to GPS")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write RTCM data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
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
