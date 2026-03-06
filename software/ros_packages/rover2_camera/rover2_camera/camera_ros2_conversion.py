import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from gi.repository import Gst, GLib


gi.require_version('Gst', '1.0')
Gst.init(sys.argv)

class CameraRos2Conversion(Node):
    def __init__(self):
        super().__init__('camera_ros2_conversion')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/d455/color/image_raw')
        self.declare_parameter('height', 480)
        self.declare_parameter('width', 640)
        self.declare_parameter('fps', 30)

        self.bridge = CvBridge()

        # Create subscription
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.listener_callback,
            10
        )

        # Load parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

    def listener_callback(self, msg: Image):
        # Convert ROS image to OpenCV BGR frame
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Initialize VideoWriter if not yet created
        if self.writer is None:
            if self.auto_resolution:
                self.height, self.width = frame.shape[:2]

        # Write frame to v4l2loopback device
        self.writer.write(frame)

    def destroy_node(self):
        # Release VideoWriter on shutdown
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"Released VideoWriter for {self.video_device}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRos2Conversion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera_ros2_conversion node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
