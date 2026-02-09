#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32  # Adjust to your actual message type
import tf2_ros
import math

class OdrivePositionOdometry(Node):
    def __init__(self):
        super().__init__('odrive_position_odometry')
        
        # ===== CONFIGURE THESE FOR YOUR ROVER =====
        self.declare_parameter('wheel_radius', 0.150)    # meters
        self.declare_parameter('track_width', 0.815)       # meters
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Track width: {self.track_width}m')
        
        # Subscribe to ODrive position topics
        # CHANGE THESE TO YOUR ACTUAL TOPICS
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
        
        # Position tracking
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.prev_left_pos = None
        self.prev_right_pos = None
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocity estimates (for odometry message)
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        self.initialized = False
        
    def left_pos_callback(self, msg):
        """Update left wheel position"""
        self.left_pos = msg.data
        self.calculate_odometry()
        
    def right_pos_callback(self, msg):
        """Update right wheel position"""
        self.right_pos = msg.data
        self.calculate_odometry()
        
    
    def calculate_odometry(self):
        """Calculate odometry from position changes"""
        
        # Initialize on first callback
        if not self.initialized:
            if self.prev_left_pos is None:
                self.prev_left_pos = self.left_pos
            if self.prev_right_pos is None:
                self.prev_right_pos = self.right_pos
            
            if self.prev_left_pos is not None and self.prev_right_pos is not None:
                self.initialized = True
                self.get_logger().info('Odometry initialized')
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt == 0:
            return
        
        # Calculate position changes
        delta_left_pos = self.left_pos - self.prev_left_pos
        delta_right_pos = self.right_pos - self.prev_right_pos
        
        # Convert to linear distances
        distance_left = delta_left_pos*self.wheel_circumference
        distance_right = delta_right_pos*self.wheel_circumference
        
        # Calculate robot motion
        distance_center = (distance_right + distance_left) / 2.0
        delta_theta = (distance_right - distance_left) / self.track_width
        
        # Update pose
        # Use midpoint method for better accuracy with large rotation changes
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
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Update previous positions and time
        self.prev_left_pos = self.left_pos
        self.prev_right_pos = self.right_pos
        self.last_time = current_time
        
    def publish_odometry(self, current_time):
        """Publish odometry message and TF transform"""
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'robot_base_origin'
        
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
        # Position covariance (tune based on your wheel slip)
        odom.pose.covariance[0] = 0.001   # x variance
        odom.pose.covariance[7] = 0.001   # y variance
        odom.pose.covariance[14] = 1e6    # z (not used)
        odom.pose.covariance[21] = 1e6    # roll (not used)
        odom.pose.covariance[28] = 1e6    # pitch (not used)
        odom.pose.covariance[35] = 0.1   # yaw variance
        
        # Velocity covariance
        odom.twist.covariance[0] = 0.001   # vx variance
        odom.twist.covariance[7] = 1e6     # vy (not used)
        odom.twist.covariance[14] = 1e6    # vz (not used)
        odom.twist.covariance[21] = 1e6    # vroll (not used)
        odom.twist.covariance[28] = 1e6    # vpitch (not used)
        odom.twist.covariance[35] = 0.01   # vyaw variance
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform
#        t = TransformStamped()
#        t.header.stamp = current_time.to_msg()
#        t.header.frame_id = 'odom'
#        t.child_frame_id = 'base_link'
#        t.transform.translation.x = self.x
#        t.transform.translation.y = self.y
#        t.transform.translation.z = 0.0
#        t.transform.rotation = odom.pose.pose.orientation
        
#        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdrivePositionOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
