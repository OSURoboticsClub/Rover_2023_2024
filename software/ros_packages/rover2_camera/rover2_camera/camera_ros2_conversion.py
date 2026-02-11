import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraRos2Conversion(Node):
    def __init__(self):
        super().__init__('camera_ros2_conversion')

        # Declare parameters
        self.declare_parameter('image_topic', '/camera/d455/color/image_raw')
        self.declare_parameter('height', 480)
        self.declare_parameter('width', 640)
        self.declare_parameter('fps', 30)
        self.declare_parameter('video_device', '/dev/video70')
        self.declare_parameter('auto_resolution', True)  # Optional: auto-detect from first frame

        self.bridge = CvBridge()
        self.writer = None  # Will initialize on first frame if auto_resolution=True

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
        self.video_device = self.get_parameter('video_device').value
        self.auto_resolution = self.get_parameter('auto_resolution').value

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

            self.writer = cv2.VideoWriter(
                self.video_device,
                cv2.CAP_V4L2,
                cv2.VideoWriter_fourcc(*'YUYV'),  # safest for loopback
                self.fps,
                (self.width, self.height)
            )

            if not self.writer.isOpened():
                self.get_logger().error(f"Failed to open video device {self.video_device}")
                self.writer = None
                return
            else:
                self.get_logger().info(f"VideoWriter opened on {self.video_device} ({self.width}x{self.height} @ {self.fps} FPS)")

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
