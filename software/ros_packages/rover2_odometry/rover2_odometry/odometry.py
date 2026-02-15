#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf2_ros
import math
from collections import deque

class OdrivePositionOdometry(Node):
    def __init__(self):
        super().__init__('odrive_position_odometry')

        # ===== CONFIGURE THESE FOR YOUR ROVER =====
        self.declare_parameter('wheel_radius', 0.150)    # meters
        self.declare_parameter('track_width', 0.815)       # meters
        self.declare_parameter('publish_rate', 10)         # Hz for odometry publishing

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Track width: {self.track_width}m')

        # Subscribe to ODrive position topics
        self.left_pos_sub = self.create_subscription(
            Float32,
            '/odrive/left/position',
            self.left_pos_callback,
            10)
        
        self.right_pos_sub = self.create_subscription(
            Float32,
            '/odrive/right/position',
            self.right_pos_callback,
            10)
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Buffer for wheel positions
        self.left_pos_buffer = deque(maxlen=1)  # Buffer for left wheel
        self.right_pos_buffer = deque(maxlen=1)  # Buffer for right wheel

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity estimates (for odometry message)
        self.vx = 0.0
        self.vth = 0.0

        # Time tracking for odometry updates
        self.last_time = self.get_clock().now()
        
        # Timer to trigger odometry publishing at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)

    def left_pos_callback(self, msg):
        """Update left wheel position"""
        self.left_pos_buffer.append(msg.data)
        
    def right_pos_callback(self, msg):
        """Update right wheel position"""
        self.right_pos_buffer.append(msg.data)

    def calculate_odometry(self):
        """Calculate odometry from position changes"""
        if not self.left_pos_buffer or not self.right_pos_buffer:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt == 0:
            return

        # Get the latest position updates
        left_pos = self.left_pos_buffer[-1]
        right_pos = self.right_pos_buffer[-1]

        # Calculate position changes
        delta_left_pos = left_pos - self.left_pos_buffer[0]
        delta_right_pos = right_pos - self.right_pos_buffer[0]

        # Convert to linear distances
        distance_left = delta_left_pos * self.wheel_circumference
        distance_right = delta_right_pos * self.wheel_circumference

        # Calculate robot motion
        distance_center = (distance_right + distance_left) / 2.0
        delta_theta = (distance_right - distance_left) / self.track_width

        # Update pose
        self.theta += delta_theta
        delta_x = distance_center * math.cos(self.theta - delta_theta / 2.0)
        delta_y = distance_center * math.sin(self.theta - delta_theta / 2.0)

        self.x += delta_x
        self.y += delta_y

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Calculate velocities for odometry message
        self.vx = distance_center / dt if dt > 0 else 0.0
        self.vth = delta_theta / dt if dt > 0 else 0.0

        self.last_time = current_time

    def publish_odometry(self):
        """Publish odometry message and TF transform"""
        self.calculate_odometry()

        # Create odometry message
        odom = Odometry()
        current_time = self.get_clock().now()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'rover_base_origin'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (convert theta to quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity in body frame
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # Covariance matrices
        odom.pose.covariance[0] = 0.001   # x variance
        odom.pose.covariance[7] = 0.001   # y variance
        odom.pose.covariance[14] = 1e6    # z (not used)
        odom.pose.covariance[21] = 1e6    # roll (not used)
        odom.pose.covariance[28] = 1e6    # pitch (not used)
        odom.pose.covariance[35] = 0.1   # yaw variance
        
        # Velocity covariance
        odom.twist.covariance[0] = 0.001   # vx variance
        odom.twist.covariance[7] = 0.001   # vy (not used)
        odom.twist.covariance[14] = 1e6    # vz (not used)
        odom.twist.covariance[21] = 1e6    # vroll (not used)
        odom.twist.covariance[28] = 1e6    # vpitch (not used)
        odom.twist.covariance[35] = 0.01   # vyaw variance
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdrivePositionOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
