import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPoint

import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.ser = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
        self.get_logger().info(f"Serial port {self.ser.port} opened at {self.ser.baudrate} baud")

        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.set_datum_srv = self.create_client(SetDatum, 'datum')
        
        self.rtcm_sub = self.create_subscription(UInt8MultiArray, 'rtcm', self.rtcm_callback, 10)

        self.buffer = bytearray()
        self.timer = self.create_timer(0.01, self.read_serial)
        
        self.datum_set = False  # Track if we've already set the datum

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

                    # Map NMEA fix quality to NavSatStatus
                    if fix_quality == 0:
                        fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
                    elif fix_quality == 1:
                        fix_msg.status.status = NavSatStatus.STATUS_FIX
                    elif fix_quality == 2:
                        fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
                    elif fix_quality in [4, 5]:
                        fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                    else:
                        fix_msg.status.status = NavSatStatus.STATUS_FIX

                    # Detect service based on NMEA sentence type
                    if parts[0].startswith('$GN'):
                        fix_msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                    else:
                        fix_msg.status.service = NavSatStatus.SERVICE_GPS

                    fix_msg.latitude = lat
                    fix_msg.longitude = lon
                    fix_msg.altitude = altitude

                    # Set position covariance based on GPS fix quality  
                    fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    fix_msg.position_covariance = self.get_gps_covariance(fix_quality)
                    
                    if fix_quality > 0:
                        self.fix_pub.publish(fix_msg)
                        self.get_logger().debug(f"Published GPS fix: lat={lat}, lon={lon}, alt={altitude}, sats={num_sats}, quality={fix_quality}")
                        
                        # Set datum if fix quality > 1 and not already set
                        if not self.datum_set:
                            self.call_set_datum(lat, lon, altitude)
                    else:
                        self.get_logger().debug(f"GPS fix: NO FIX STATUS")

            except (ValueError, IndexError) as e:
                self.get_logger().warn(f"Failed to parse NMEA line: {line}, error: {e}")

    def get_gps_covariance(self, fix_quality):
        """Get position covariance based on GPS fix quality"""
        if fix_quality in [4, 5]:
            # RTK 0.01-0.5m
            return [
                0.01, 0.0, 0.0, 
                0.0, 0.01, 0.0, 
                0.0, 0.0, 0.04
            ]
        elif fix_quality == 2:
            # DGPS 0.5-2m
            return [
                1.0, 0.0, 0.0, 
                0.0, 1.0, 0.0, 
                0.0, 0.0, 4.0
            ]
        elif fix_quality == 1:
            # Standard GPS 2-5m 
            return [
                5.0, 0.0, 0.0, 
                0.0, 5.0, 0.0, 
                0.0, 0.0, 10.0
            ]
        else:
            # No Fix 
            return [
                1e6, 0.0, 0.0, 
                0.0, 1e6, 0.0, 
                0.0, 0.0, 1e6
            ]
        

    def call_set_datum(self, lat, lon, alt):
        """Call SetDatum service to set the datum origin"""
        if not self.set_datum_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SetDatum service not available')
            return
        
        req = SetDatum.Request()
        req.geo_pose.position.latitude = lat
        req.geo_pose.position.longitude = lon
        req.geo_pose.position.altitude = alt
        
        future = self.set_datum_srv.call_async(req)
        future.add_done_callback(self.datum_response_callback)
        
        self.get_logger().info(f"Calling SetDatum with lat={lat}, lon={lon}, alt={alt}")

    def datum_response_callback(self, future):
        """Handle SetDatum service response"""
        try:
            response = future.result()
            self.datum_set = True
            self.get_logger().info('SetDatum service call successful - datum has been set')
        except Exception as e:
            self.get_logger().error(f'SetDatum service call failed: {e}')

    def rtcm_callback(self, msg: UInt8MultiArray):
        """Write incoming RTCM bytes to GPS"""
        try:
            data = bytes(msg.data)
            self.ser.write(data)
            self.get_logger().info(f"Wrote {len(data)} RTCM bytes to GPS")
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
