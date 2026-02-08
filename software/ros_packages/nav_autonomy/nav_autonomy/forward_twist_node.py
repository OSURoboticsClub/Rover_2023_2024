import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rover2_control_interface.msg import DriveCommandMessage

class ForwardTwistNode(Node):
    def __init__(self):
        super().__init__('forward_twist_node')
        queue_size = 10
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            queue_size
        )

        self.pub = self.create_publisher(
            DriveCommandMessage,
            '/command_control/ground_station_drive',
            queue_size
        )

    def twist_callback(self, msg):
        drive_msg: DriveCommandMessage = DriveCommandMessage()
        drive_msg.drive_twist = msg
        drive_msg.controller_present = False
        drive_msg.ignore_drive_control = False
        self.pub.publish(drive_msg)

def main():
    rclpy.init()
    node = ForwardTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
