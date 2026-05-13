#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point

class OdomToTwist(Node):
    def __init__(self):
        super().__init__('odom_to_twist_node')
        
        self.declare_parameter('first_goal_linear', 0)
        self.declare_parameter('second_goal_linear', 2)
        self.declare_parameter('max_velo',0.45)
        self.declare_parameter('max_accel',0.15)
        self.declare_parameter('completion_accuracy', 0.01)
        self.declare_parameter('slip_level', 'low')
        self.declare_parameter('test_type', 'x-linear')
        self.declare_parameter('num_tests',100)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.odom_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.first_goal_linear = self.get_parameter('first_goal_linear').value
        self.second_goal_linear = self.get_parameter('second_goal_linear').value
        self.max_velo = self.get_parameter('max_velo').value
        self.completion_accuracy = self.get_parameter('completion_accuracy').value
        self.slip_level = self.get_parameter('slip_level').value
        self.max_accel = self.get_parameter('max_accel').value
        self.test_type = self.get_parameter('test_type').value
        self.num_tests = self.get_parameter('num_tests').value
        
        self.get_logger().info("Odom to Twist node has started.")
        
        self.current_velo = 0
        self.executed_tests = 0
        self.current_goal = self.first_goal_linear
        
    def odom_callback(self, msg: Odometry):
        twist_msg = Twist()
        pose = msg.pose.pose
        
        if(self.test_type == "x-linear"):
            twist_msg = self.trap_motion_x(pose.position,self.current_goal)
            
        self.cmd_pub.publish(twist_msg)
        self.get_logger().debug(f"Published Twist: {twist_msg}")
        
    def trap_motion_x(self, start_point: Point, goal_dist: int):
        twist_msg = Twist()
        dx = goal_dist-start_point.x
        distance = abs(dx)
        direction = 1 if dx >= 0 else -1
        
        if(distance < self.completion_accuracy and self.executed_tests < self.num_tests):
            if(self.current_goal == self.first_goal_linear):
                self.current_goal = self.second_goal_linear
            else:
                self.current_goal = self.first_goal_linear
            self.executed_tests += 1
            self.get_logger().info(f"{self.executed_tests}/{self.num_tests} tests done")
            
        d_decel = (self.current_velo**2) / (2*self.max_accel)
        desired_speed = 0
        
        if distance > d_decel:
            desired_speed = min(self.current_velo + self.max_accel * 0.1, self.max_velo)
        else:
            desired_speed = max(self.current_velo - self.max_accel * 0.1, 0.0)
            
        twist_msg.linear.x = desired_speed * direction
        self.current_velo = desired_speed
        
        return twist_msg

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
