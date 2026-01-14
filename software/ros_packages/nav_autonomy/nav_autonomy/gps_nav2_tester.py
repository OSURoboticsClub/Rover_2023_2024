#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
import time

class GPSNav2Tester(Node):
    def __init__(self):
        super().__init__('gps_nav2_tester')

        # Publisher to Nav2 goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Static GPS points (lat, lon)
        self.gps_points = [
            (37.4276, -122.1696),
            (37.4277, -122.1695),
            (37.4278, -122.1694)
        ]

        # Index to track which point we're on
        self.current_index = 0
        self.timer_period = 5.0  # seconds between goals
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Optional: frame of your GPS origin (usually "gps_link")
        self.gps_frame = "gps_link"
        self.map_frame = "map"

    def timer_callback(self):
        if self.current_index >= len(self.gps_points):
            self.get_logger().info("All GPS points sent. Shutting down...")
            rclpy.shutdown()
            return

        lat, lon = self.gps_points[self.current_index]

        # Create a NavSatFix-like pose (you could also publish actual NavSatFix messages)
        gps_pose = PoseStamped()
        gps_pose.header.stamp = self.get_clock().now().to_msg()
        gps_pose.header.frame_id = self.gps_frame
        gps_pose.pose.position.x = lat   # temporarily store lat in x
        gps_pose.pose.position.y = lon   # temporarily store lon in y
        gps_pose.pose.position.z = 0.0
        gps_pose.pose.orientation.w = 1.0

        try:
            # Transform from GPS frame to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.gps_frame,
                rclpy.time.Time()
            )
            map_pose = do_transform_pose(gps_pose, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return  # try again on next timer tick

        # Publish PoseStamped to Nav2
        self.goal_pub.publish(map_pose)
        self.get_logger().info(f"Sent goal {self.current_index}: x={map_pose.pose.position.x:.2f}, y={map_pose.pose.position.y:.2f}")

        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = GPSNav2Tester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

