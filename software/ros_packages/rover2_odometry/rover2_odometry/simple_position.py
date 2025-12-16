import rclpy
from rclpy.node import Node
from rover2_control_interface.msg import DriveCommandMessage
from rover2_control_interface.msg import GPSStatusMessage
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import String

class SimplePosition(Node):
    current_heading = None
    current_speed = None
    current_latitude = None
    current_longitude = None
    latest_gps_latitude = None
    latest_gps_longitude = None
    pwm_to_meter_per_sec_multiplier = 2.3 # 2 m/s at 100% power
    current_speed = 0.0

    def __init__(self):
        super().__init__('simple_position')
        self.get_logger().info('Node has started')
        self.gps_subscription = self.create_subscription(GPSStatusMessage, 'tower/status/gps', self.gps_listener_callback, 10)
        self.imu_subscription = self.create_subscription(Float32, 'imu/data/heading', self.imu_heading_listener_callback, 10)
        self.drive_subscription = self.create_subscription(DriveCommandMessage, 'command_control/iris_drive', self.drive_listener_callback, 10)
        self.drive_subscription2 = self.create_subscription(DriveCommandMessage, 'command_control/ground_station_drive', self.drive_listener_callback, 10)
        self.manual_position_subscription = self.create_subscription(String, 'autonomous/manually_set_position', self.manual_set_position_listener_callback, 10)

        self.publisher = self.create_publisher(String, 'autonomous/simple_position', 10)
        #self.current_latitude = 44.566973
        #self.current_longitude = -123.274295
        #self.current_heading = 45.0
        #self.current_speed = 1.0

        self.control_timer = self.create_timer(1.0, self.publish_loop)
        self.gps_midpoint_timer = self.create_timer(3.3, self.gps_midpoint_loop)

    def gps_midpoint_loop(self):
        if self.current_latitude == None:
            return
        if self.latest_gps_latitude == None:
            return 
        self.get_logger().info("GPS midpoint loop!")
        mid_lat, mid_lon = self.midpoint(self.latest_gps_latitude, self.latest_gps_longitude, self.current_latitude, self.current_longitude)
        self.current_latitude = mid_lat
        self.current_longitude = mid_lon

    def publish_loop(self):
        # calculate new position
        if self.current_latitude == None:
            self.get_logger().info("Position not set")
            return
        if self.current_heading == None:
            self.get_logger().info("Heading not set")
            return
        distance_covered = 1.0 * self.current_speed
        geod = Geodesic.WGS84

        #self.get_logger().info(f"Current speed: {self.current_speed}. Current heading: {self.current_heading}")
        new_pos = geod.Direct(self.current_latitude, self.current_longitude, self.current_heading, distance_covered)
        self.current_latitude = new_pos['lat2']
        self.current_longitude = new_pos['lon2']

        #self.get_logger().info(f'Distance covered: {distance_covered}. New lat: {self.current_latitude}, new lon: {self.current_longitude}')
        ros_msg = String()
        ros_msg.data = str(self.current_latitude) + ";" + str(self.current_longitude)
        self.publisher.publish(ros_msg)

    def midpoint(self, lat1, lon1, lat2, lon2):
        geod = Geodesic.WGS84

        # Compute the geodesic inverse problem
        inv_result = geod.Inverse(lat1, lon1, lat2, lon2)

        # Get the midpoint using the direct geodesic method
        midpoint_result = geod.Direct(lat1, lon1, inv_result['azi1'], inv_result['s12'] * 0.4)

        return midpoint_result['lat2'], midpoint_result['lon2']

    def gps_listener_callback(self, msg):
        self.latest_gps_latitude = msg.rover_latitude
        self.latest_gps_longitude = msg.rover_longitude

        if self.current_latitude is None:
            self.current_latitude = msg.rover_latitude
            self.current_longitude = msg.rover_longitude
            self.get_logger().info("Just set starting position!")

        # if self.current_latitude is not None:
        #     mid_lat, mid_lon = self.midpoint(self.latest_gps_latitude, self.latest_gps_longitude, self.current_latitude, self.current_longitude)
        #     self.current_latitude = mid_lat
        #     self.current_longitude = mid_lon
        # else:
        #     self.current_latitude = self.latest_gps_latitude
        #     self.current_longitude = self.latest_gps_longitude  

    def imu_heading_listener_callback(self, msg):
        self.current_heading = msg.data

    def drive_listener_callback(self, msg):
        #self.get_logger().info(f"Received twist, controller present = {msg.controller_present}")
        if msg.controller_present:
            speed = msg.drive_twist.linear.x * self.pwm_to_meter_per_sec_multiplier
            self.get_logger().info(f"Setting speed to {speed}")
            self.current_speed = speed
        
    def manual_set_position_listener_callback(self, msg):
        parts = msg.data.split(';')
        lat = float(parts[0])
        lon = float(parts[1])
        self.get_logger().info(f'Received: "{msg.data}" on manually_set_position')
        self.current_latitude = lat
        self.current_longitude = lon

def main(args=None):
    rclpy.init(args=args)
    node = SimplePosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
