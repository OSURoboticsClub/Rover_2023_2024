from datetime import datetime

# from image_transport_py import ImageTransport
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import time
from sensor_msgs.msg import Image

latest_image = None

class MySubscriber(Node):
    def __init__(self):
        super().__init__('spectrometry_subscriber')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        self.image = None
        self.create_timer(0.1, self.display_image)


    def image_callback(self, msg):

        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        timestamp = sec + nanosec * 1e-9
        readable_time = datetime.fromtimestamp(timestamp)
        formatted_time = readable_time.strftime('%Y-%m-%d-%H-%M-%S')

        self.get_logger().info('got a new image from frame_id:=%s' % msg.header.frame_id)
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imwrite((msg.header.frame_id + '-' + formatted_time + '.png'), self.image)

    def display_image(self):
        # This runs in the main thread → safe for OpenCV GUI
        if self.image is not None:
            cv2.imshow("image", self.image)
            cv2.waitKey(1)  # Process GUI events


def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()