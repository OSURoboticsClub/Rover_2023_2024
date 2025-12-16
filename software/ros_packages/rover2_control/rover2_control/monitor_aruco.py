import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import cv2.aruco as aruco
import time


class monitor_aruco(Node):
    def __init__(self, camera_devices):
        super().__init__('aruco_scanner_from_devices')

        self.captures = {}
        self.pubs = {}

        for label, device_path in camera_devices.items():
            cap = cv2.VideoCapture(device_path)
            if not cap.isOpened():
                self.get_logger().error(f"Could not open camera at {device_path}")
                continue
            self.captures[label] = cap
            topic = f"/{label}/aruco_id"
            self.pubs[label] = self.create_publisher(Int32, topic, 10)
            self.get_logger().info(f"Opened camera {device_path} as '{label}' → {topic}")

        # ArUco detection setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.create_timer(1.0, self.process_cameras)

    def process_cameras(self):
        for label, cap in self.captures.items():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"Failed to read from camera '{label}'")
                continue

            aruco_id = self.detect_aruco_id(frame)
            msg = Int32()
            msg.data = aruco_id
            self.pubs[label].publish(msg)
            #self.get_logger().info(f"[{label}] ArUco ID: {aruco_id}")

    def detect_aruco_id(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            return int(ids[0])
        return -1


def main(args=None):
    rclpy.init(args=args)

    # Map of camera labels to device paths
    camera_devices = {
        "infrared": "/dev/video20",
        "tower": "/dev/video22",
	"gripper": "/dev/video21"
    }

    node = monitor_aruco(camera_devices)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
