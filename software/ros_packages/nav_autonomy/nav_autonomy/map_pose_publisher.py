import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


# This node is necessary to convert global ekf published odometry into message format expected by rtab for map alignment


class MapPosePublisher(Node):

    def __init__(self):
        super().__init__('map_pose_publisher')

        self.sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.callback,
            10)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/global_pose',
            10)

    def callback(self, msg: Odometry):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        pose_msg.pose.pose.orientation = Quaternion()
        self.pub.publish(pose_msg)

def main():
    rclpy.init()
    node = MapPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
