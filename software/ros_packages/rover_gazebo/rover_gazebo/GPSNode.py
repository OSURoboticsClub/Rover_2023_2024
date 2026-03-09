import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.set_datum_srv = self.create_client(SetDatum, 'datum')
        
        self.raw_gps_sub = self.create_subscription(NavSatFix, 'gps/raw', self.add_covariance, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.buffer = bytearray()
        # self.timer = self.create_timer(0.01, self.add_covarience)
        self.imu_quat = Quaternion()
        self.datum_set = False  # Track if we've already set the datum

    def add_covariance(self, msg):
        nav_msg = msg
        nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        nav_msg.position_covariance = self.get_gps_covariance(1)
        self.fix_pub.publish(nav_msg)
        if not self.datum_set:
            self.call_set_datum(nav_msg.latitude, nav_msg.longitude, nav_msg.altitude)

        return
    def imu_callback(self, msg):
        self.imu_quat = msg.orientation

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
        req.geo_pose.orientation = self.imu_quat
	        

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
