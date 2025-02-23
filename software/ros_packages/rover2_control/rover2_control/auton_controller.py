import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage
from rover2_control_interface.msg import GPSStatusMessage
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geographiclib.geodesic import Geodesic
from transforms3d.euler import quat2euler
from dataclasses import dataclass
import math
import json
from dataclasses import asdict

@dataclass
class Location:
    latitude: float
    longitude: float

class auton_controller(Node):
    waypoint_destination = None
    subpoints = None
    curr_destination = None
    rover_position = Location(44.56726, -123.27363)
    current_heading = 0.0
    target_heading = None
    state = "stopped"
    control_timer = None 
    offset = None
    time_driving = 0.0

    # 30%: 117.5" in 5 sec
    # 20%: 

    def __init__(self):
        super().__init__('auton_controller')
        self.control_subscription = self.create_subscription( String, 'auton_control', self.control_listener_callback, 10)
        self.gps_subscription = self.create_subscription(GPSStatusMessage, 'tower/status/gps', self.gps_listener_callback, 10)
        #self.imu_subscription = self.create_subscription(Imu, 'imu/data', self.imu_listener_callback, 10)
        self.imu_subscription = self.create_subscription(Float32, 'imu/data/heading', self.imu_heading_listener_callback, 10)

        self.response_publisher = self.create_publisher(String, 'auton_control_response', 10)
        self.drive_publisher = self.create_publisher(DriveCommandMessage, 'command_control/ground_station_drive', 10)


    def publish_drive_message(self, linear_speed, angular_speed):
        """Publish the current linear and angular speed to drivetrain"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angular_speed
        custom_msg = DriveCommandMessage()
        custom_msg.controller_present = True
        custom_msg.ignore_drive_control = False
        custom_msg.drive_twist = twist_msg
        self.drive_publisher.publish(custom_msg)

    def get_target_heading(self, target):
        geod = Geodesic.WGS84
        lat1 = self.rover_position.latitude
        lon1 = self.rover_position.longitude
        lat2 = target.latitude
        lon2 = target.longitude
        result = geod.Inverse(lat1, lon1, lat2, lon2)
        return result['azi1']

    def publish_log_msg(self, text):
        msg = String()
        msg.data = text
        self.response_publisher.publish(msg)

    def get_distance_to_location(self, target):
        geod = Geodesic.WGS84
        lat1 = self.rover_position.latitude
        lon1 = self.rover_position.longitude
        lat2 = target.latitude
        lon2 = target.longitude
        result = geod.Inverse(lat1, lon1, lat2, lon2)
        return result['s12'] * 3.28084  # Convert meters to feet because this is America

    def get_heading_error(self):
        """Returns the shortest signed heading error in degrees."""
        error = self.target_heading - self.current_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        return error

    def compute_curvature(self, target):
        dist_to_target = self.get_distance_to_location(target)

        # Compute lateral error (y)
        heading_error = math.radians(self.get_heading_error())  # Convert to radians
        y = dist_to_target * math.sin(heading_error)  # Perpendicular distance

        # Compute curvature
        if dist_to_target == 0:
            return 0  # Prevent division by zero
        curvature = (2 * y) / (dist_to_target ** 2)
        return curvature

    def get_points_along_line(self, step_feet=15):
        """
        Moves along the geodesic path from (lat1, lon1) to (lat2, lon2) 
        in steps of `step_feet`, returning a list of coordinates
        """
        total_distance = self.get_distance_to_location(self.waypoint_destination)
        geod = Geodesic.WGS84
        lat1 = self.rover_position.latitude
        lon1 = self.rover_position.longitude

        self.subpoints = []  # Exclude start and end positions
        self.get_logger().info("Distance: " + str(total_distance))

        for i in range(1, 100):
            traveled_distance = i * step_feet
            self.get_logger().info("Traveled: " + str(traveled_distance))
            if traveled_distance >= total_distance - step_feet/2.0:
                break
            new_pos = geod.Direct(lat1, lon1, self.target_heading, traveled_distance * 1/3.28084)
            self.subpoints.append(Location(new_pos['lat2'], new_pos['lon2']))
        
        msg = "subpoints;" + json.dumps([asdict(loc) for loc in self.subpoints])
        self.publish_log_msg(msg)

    def set_next_dest(self):
        if self.subpoints is not None and len(self.subpoints) > 0:
            self.curr_destination = self.subpoints[0]
            self.subpoints.pop(0)
        elif self.waypoint_destination is not None:
            self.curr_destination = self.waypoint_destination
            self.waypoint_destination = None
        else:
            self.curr_destination = None
        self.get_logger().info("Set new dest: " + str(self.curr_destination))
        if self.curr_destination is not None:
            self.publish_log_msg("nextdest;" + str(self.curr_destination.latitude) + ";" + str(self.curr_destination.longitude))
        
    def control_loop(self):
        if self.state == "stopped":
            self.publish_drive_message(0.0, 0.0)
            self.control_timer.cancel()
            self.control_timer = None
            self.subpoints = None
            self.publish_log_msg("Stopped autonomous control")
            return

        #heading_error = self.get_heading_error()
        self.publish_drive_message(0.2,0.0)
        self.time_driving += 0.1
        print("Time driving: " + str(self.time_driving))
        if self.time_driving >= 5.0:
            self.state = "stopped"
        return

        if self.state == "turning":
            self.get_logger().info("Turning. Target: " + f"{self.target_heading:.1f}" + ". Current: " + f"{self.current_heading:.1f}" + ", Error: " + f"{heading_error:.1f}")
            if abs(heading_error) < 2.5:  # Example threshold
                self.get_logger().info("Target heading reached.")
                self.publish_log_msg("Reached target heading. Now driving")
                self.state = "driving"
            else:
                angular_speed = 0.4 # rad/s
                if heading_error > 0:
                    angular_speed *= -1
                if abs(heading_error) < 30: # slow down on approach
                    angular_speed *= 0.6
                self.publish_drive_message(0.0, angular_speed) 

        elif self.state == "driving":
            self.target_heading = self.get_target_heading(self.curr_destination)
            distance_to_nearest_point = self.get_distance_to_location(self.curr_destination)
            distance_to_waypoint = self.get_distance_to_location(self.waypoint_destination)
            curv = self.compute_curvature(self.curr_destination)


            if distance_to_nearest_point < 11.0:
                self.set_next_dest()
                if self.curr_destination is None:
                    self.state = "stopped"
                    self.get_logger().info("Reached destination. Stopping...")
                    return

            linear = 0.3
            angular = -curv * linear
            if angular > 0.6:
                angular = 0.6
            elif angular < -0.6:
                angular = -0.6
            
            log1 = "Driving. Dist to target: " + f"{distance_to_nearest_point:.0f}. Curv: " + f"{curv:0.1f}" + ". Angular: " + str(angular) + ". "
            heading_log = "Target H: " + f"{self.target_heading:.1f}, " + "Current H: " + f"{self.current_heading:.1f}"
            self.get_logger().info(log1 + heading_log)
            self.publish_drive_message(linear, angular)

    def control_listener_callback(self, msg):
        """Listens to auton_control topic for commands"""
        self.get_logger().info(f'Received: "{msg.data}" on auton_control')

        try:
            parts = msg.data.split(';')
            command = parts[0]
            lat = float(parts[1])
            lon = float(parts[2])

            if command == "GOTO":
                self.get_logger().info(f"Command GOTO received with target lat: {lat}, lon: {lon}")
                self.state = "turning"
                self.waypoint_destination = Location(lat, lon)
                self.target_heading = self.get_target_heading(self.waypoint_destination)
                self.get_points_along_line()
                self.set_next_dest()
                if self.control_timer is not None:
                    self.control_timer.cancel()
                self.control_timer = self.create_timer(0.1, self.control_loop)
            elif command == "STOP":
                self.get_logger().info("STOP command received. Stopping autonomous navigation.")
                self.state = "stopped"
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse message: {msg.data}. Error: {e}")

    def imu_listener_callback(self, msg):
        """Receive IMU data, convert to Euler Angles"""
        quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

        euler = quat2euler(quat, axes='sxyz')  # 'sxyz' is a common rotation convention
        roll, pitch, yaw = euler
        roll_degrees = roll * 57.2958

        if self.offset == None:
            self.current_heading = 18.0  # The rover should start pointing in alignment with Merryfield
            self.get_logger().info(f'Heading should be 18')
            self.get_logger().info(f'Current heading is: ' + str(roll_degrees))
            self.get_logger().info(f'Thus the offset is: ' + str(18.0 - roll_degrees))
            self.offset = 18.0 - roll_degrees # At the start, determine the offset in degrees of the rover if the IMU is off
        else:
            new_heading = roll_degrees + 180.0
            if new_heading > 180.0:
                new_heading -= 360.0
            #self.get_logger().info(f'heading is: ' + str(new_heading))
            self.current_heading = new_heading
            # if(self.current_heading < -180.0):
            #     self.current_heading += 360
            # if(self.current_heading > 180.0):
            #     self.current_heading -= 360

    def imu_heading_listener_callback(self, msg):
        """Listens to auton_control topic for commands"""
        self.current_heading = msg.data
        #self.get_logger().info(f"Received heading: " + str(self.current_heading))

    def gps_listener_callback(self, msg):
        """Receive GPS data"""
        self.rover_position.latitude = msg.rover_latitude
        self.rover_position.longitude = msg.rover_longitude

def main(args=None):
    rclpy.init(args=args)
    node = auton_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
