#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

class OdrivePositionOdometry(Node):
    def __init__(self):
        super().__init__('odrive_position_odometry')
        
        # ===== CONFIGURE THESE FOR YOUR ROVER =====
        self.declare_parameter('wheel_radius', 0.150)    # meters
        self.declare_parameter('track_width', 0.815)     # meters
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Track width: {self.track_width}m')
        
        # Subscribe to ODrive state topic and publish odometry
        self.create_subscription(JointState, '/odrive/motor_states', self.motor_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # Position tracking
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.initialized = False
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0


    def motor_state_callback(self, msg):
        states = { name: (pos, vel) for name, pos, vel in zip(msg.name, msg.position, msg.velocity) }
        left_state = states.get('left_wheel')
        right_state = states.get('right_wheel')

        if left_state is None or right_state is None:
            self.get_logger().warn('Missing left or right wheel state in JointState message', throttle_duration_sec=1.0)
            return

        if not self.initialized:
            self.prev_left_pos = left_state[0]
            self.prev_right_pos = right_state[0]
            self.initialized = True
            return
        
        delta_left = (left_state[0] - self.prev_left_pos) * self.wheel_radius
        delta_right = (right_state[0] - self.prev_right_pos) * self.wheel_radius

        self.prev_left_pos = left_state[0]
        self.prev_right_pos = right_state[0]

        dist_center = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.track_width

        # 2nd order Runge-Kutta is more accurate??
        self.theta += delta_theta / 2.0
        self.x += dist_center * math.cos(self.theta)
        self.y += dist_center * math.sin(self.theta)
        self.theta += delta_theta / 2.0

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        v_left = left_state[1] * self.wheel_radius
        v_right = right_state[1] * self.wheel_radius
        self.vx = (v_left + v_right) / 2.0
        self.vth = (v_right - v_left) / self.track_width
        self.publish_odometry(msg.header.stamp)

        
    def publish_odometry(self, stamp):
        """Publish odometry message and TF transform"""
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = stamp
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
        # Position covariance (tune based on your wheel slip)
        odom.pose.covariance[0] = 0.01   # x variance
        odom.pose.covariance[7] = 0.01   # y variance
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
