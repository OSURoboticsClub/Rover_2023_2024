#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose_stamped
import time
import utm

class GPSNav2Tester(Node):
    def __init__(self):
        super().__init__('gps_nav2_tester')

        # Publisher to Nav2 goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/gps_test_pose', 10)

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Static MU Quad GPS points (lat, lon)
        self.gps_points = [
            (44.565733876856484, -123.27892458129588),
            (44.565763603605404, -123.27888949938009),
            (44.565800086801396, -123.27882880792261)
        ]

        # Index to track which point we're on
        self.current_index = 0
        self.timer_period = 5.0  # seconds between goals
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.utm_frame = "utm"
        self.map_frame = "map"

    def timer_callback(self):
        if self.current_index >= len(self.gps_points):
            self.current_index = 0  # Reset to first point or stop
            self.get_logger().info("All GPS points sent. Restarting")
            return

        lat, lon = self.gps_points[self.current_index]
        east, north, zone_number, zone_letter = utm.from_latlon(lat, lon)

        # Create a PoseStamped in the UTM frame (metric coordinates)
        utm_pose = PoseStamped()
        utm_pose.header.stamp = self.get_clock().now().to_msg()
        utm_pose.header.frame_id = self.utm_frame
        utm_pose.pose.position.x = east
        utm_pose.pose.position.y = north
        utm_pose.pose.position.z = 0.0
        utm_pose.pose.orientation.w = 1.0

        try:
            # Transform from utm frame to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time()
            )
            
            map_pose = do_transform_pose_stamped(utm_pose, transform)
            map_pose.header.stamp = self.get_clock().now().to_msg()
            map_pose.header.frame_id = self.map_frame
            self.goal_pub.publish(map_pose)
            self.get_logger().info(f"Sent goal {self.current_index}: x={map_pose.pose.position.x:.2f}, y={map_pose.pose.position.y:.2f}")
            self.current_index += 1

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    node = GPSNav2Tester()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

