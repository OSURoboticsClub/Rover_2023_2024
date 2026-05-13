import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('device', '/dev/rover/camera_infrared')
        self.declare_parameter('fps', 30)
        self.declare_parameter('aruco_dict_type', cv2.aruco.DICT_4X4_50)

        device = self.get_parameter('device').get_parameter_value().string_value
        fps = self.get_parameter('fps').value
        self.aruco_dict_type = self.get_parameter('aruco_dict_type').value

        # Camera
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {device}')
            raise RuntimeError(f'Could not open video device: {device}')

        self.get_logger().info(f'Opened camera: {device}')

        self.drive_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info('Aruco Detector Node started.')
        self.no_count = 0
        self.aruco_found = False
    def timer_callback(self):
        ret, image = self.cap.read()
        if self.aruco_found:
            return

        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        center_x_pct, center_y_pct, marker_width_pct = self.detect_first_aruco_marker(image)
        twist = Twist()
        if center_x_pct is None:
            self.no_count +=1
            if(self.no_count > 60):
                twist.angular.z = 1.0
                self.drive_publisher.publish(twist)
            return
        
        if marker_width_pct > 0.11:
            self.drive_publisher.publish(twist)
            self.aruco_found = True
            return
            
        self.no_count = 0
        twist.linear.x = 1-abs(0.5-(center_x_pct))
        twist.angular.z = 0.5-center_x_pct
        self.drive_publisher.publish(twist)
        
        self.get_logger().info(
            f'Marker — center_x: {center_x_pct:.3f}, center_y: {center_y_pct:.3f}, width_pct: {marker_width_pct:.3f}'
        )

    def detect_first_aruco_marker(self, image):
        if image is None:
            self.get_logger().warn('Image is None')
            return None, None, None

        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None or len(corners) == 0:
            self.get_logger().debug('No ArUco markers detected')
            return None, None, None

        img_width = gray.shape[1]
        img_height = gray.shape[0]

        corner = corners[0]
        img_corners = corner[0]
        top_left = img_corners[0]
        bottom_right = img_corners[2]

        distance_pxls = self.get_distance(top_left, bottom_right)
        marker_width_pct = distance_pxls / img_width
        
        center_x_pct, center_y_pct = self.get_marker_center_percentage(
            corner[0], img_width, img_height
        )

        return center_x_pct, center_y_pct, marker_width_pct

    def get_marker_center_percentage(self, corner_points, img_width, img_height):
        center_x = np.mean(corner_points[:, 0])
        center_y = np.mean(corner_points[:, 1])
        return center_x / img_width, center_y / img_height

    def get_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info('Camera released.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
