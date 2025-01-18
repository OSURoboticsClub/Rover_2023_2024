import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Use appropriate message type

class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('auton_control_node')
        
        self.subscription = self.create_subscription(
            String,
            'AutonomousControl',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(String, 'ReplyTopic', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}" on AutonomousControl')

        reply_msg = String()
        reply_msg.data = f'Starting Autonomous'
        self.publisher.publish(reply_msg)
        self.get_logger().info(f'Published: "{reply_msg.data}" to ReplyTopic')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()