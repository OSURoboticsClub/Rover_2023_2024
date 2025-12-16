#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from shape_msgs.msg import Plane
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # Parameters
        self.declare_parameter("image_topic", "/camera/d405/color/image_rect_raw")
        self.declare_parameter("marker_length", 0.05)
        self.declare_parameter("camera_frame", "d405_color_optical_frame")
        self.declare_parameter("ground_frame", "world")
        self.declare_parameter("intersection_radius", 0.20)
        self.declare_parameter("intersection_height", 0.10)
        self.declare_parameter("plane_topic", "ground_plane")

        self.image_topic = self.get_parameter("image_topic").value
        self.marker_length = self.get_parameter("marker_length").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.ground_frame = self.get_parameter("ground_frame").value
        self.intersection_radius = self.get_parameter("intersection_radius").value
        self.intersection_height = self.get_parameter("intersection_height").value
        self.plane_topic = self.get_parameter("plane_topic").value

        self.bridge = CvBridge()

        # Intrinsics from camera_info
        self.camera_matrix = np.array(
            [
                [433.28924560546875, 0.0, 423.25732421875],
                [0.0, 432.7252197265625, 239.83599853515625],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        # Distortion coefficients (plumb_bob: k1, k2, t1, t2, k3)
        self.dist_coeffs = np.array(
            [
                [-0.054275549948215485],
                [0.0626617893576622],
                [-8.124048326862976e-05],
                [0.0009501233580522239],
                [-0.02102944254875183],
            ],
            dtype=np.float32,
        )

        # ArUco dictionary and detector parameters (DICT_4X4_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "aruco/pose", 10)
        self.pose_ground_pub = self.create_publisher(
            PoseStamped, "aruco/pose_ground", 10
        )
        self.pt_pub = self.create_publisher(
            PointStamped, "aruco/ground_intersection", 10
        )
        self.marker_pub = self.create_publisher(Marker, "aruco/markers", 10)

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.plane_sub = self.create_subscription(
            Plane, self.plane_topic, self.plane_callback, 10
        )

        self.plane_srv = self.create_service(
            Trigger, "set_ground_plane", self.set_plane_callback
        )

        self.plane_coeffs_current = None  # np.array([a, b, c, d])
        self.plane_coeffs = None  # np.array([a, b, c, d])
        self.has_plane = False

        # TF buffer & listener (for camera_frame -> ground_frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"ArUco detector started. Subscribing to {self.image_topic}, "
            f"marker_length={self.marker_length}, ground_frame={self.ground_frame}"
        )

    def plane_callback(self, msg: Plane) -> None:
        """Store plane coefficients a, b, c, d from Plane.msg (shape_msgs/Plane.coef)."""
        if len(msg.coef) != 4:
            self.get_logger().warn(
                "Plane.msg must contain 4 coefficients [a, b, c, d]"
            )
            return
        self.plane_coeffs_current = np.array(msg.coef, dtype=np.float32)
        # self.has_plane = False

    def image_callback(self, msg: Image) -> None:
        if not self.has_plane:
            # No plane yet; nothing to intersect against
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        if ids is None or len(ids) == 0:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs,
        )

        # Take first marker
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        # Rotation & quaternion in camera frame
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R_cam_marker)

        # Pose in camera_frame
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        # Ray in camera frame (marker +Y axis)
        y_dir_cam = R_cam_marker[:, 1].astype(np.float32)
        origin_cam = tvec.astype(np.float32)

        # Transform ray to ground_frame
        origin_base, dir_base = self._to_ground_frame(origin_cam, y_dir_cam)
        if origin_base is None:
            return

        # Pose in ground_frame
        self._publish_pose_ground(origin_cam, R_cam_marker, msg)

        # Intersect with plane and publish cylinder
        P = self._intersect_ray_with_plane(origin_base, dir_base)
        if P is not None:
            self._publish_intersection(P, msg)

    def set_plane_callback(self, request, response):
        """Service to set the ground plane
        
        """
        if self.plane_coeffs_current is None:
            response.success = False
            response.message = "No plane received yet."
            return response

        self.plane_coeffs = self.plane_coeffs_current.copy()
        self.has_plane = True

        response.success = True
        response.message = (
            f"Ground plane set to coefficients: {self.plane_coeffs.tolist()}"
        )
        self.get_logger().info(response.message)
        return response


    def _to_ground_frame(self, origin_cam: np.ndarray, dir_cam: np.ndarray):
        """Transform point and direction from camera_frame to ground_frame."""
        try:
            # Use latest TF available to avoid extrapolation into the future on bag playback
            tf = self.tf_buffer.lookup_transform(
                self.ground_frame,
                self.camera_frame,
                Time()  # latest
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.camera_frame}->{self.ground_frame} failed: {e}"
            )
            return None, None

        t = tf.transform.translation
        q = tf.transform.rotation

        R_base_cam = self._quaternion_to_rotation_matrix(
            q.x, q.y, q.z, q.w
        )
        t_base_cam = np.array([t.x, t.y, t.z], dtype=np.float32)

        origin_base = R_base_cam @ origin_cam + t_base_cam
        dir_base = R_base_cam @ dir_cam  # direction only rotates

        return origin_base, dir_base

    def _publish_pose_ground(
        self, origin_cam: np.ndarray, R_cam_marker: np.ndarray, src_msg: Image
    ):
        """Publish ArUco pose in ground_frame (world/base_link)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.ground_frame,
                self.camera_frame,
                Time()  # latest
            )
        except Exception:
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        R_ground_cam = self._quaternion_to_rotation_matrix(
            q.x, q.y, q.z, q.w
        )
        t_ground_cam = np.array([t.x, t.y, t.z], dtype=np.float32)

        origin_ground = R_ground_cam @ origin_cam + t_ground_cam
        R_ground_marker = R_ground_cam @ R_cam_marker
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R_ground_marker)

        msg = PoseStamped()
        msg.header.stamp = src_msg.header.stamp
        msg.header.frame_id = self.ground_frame
        msg.pose.position.x = float(origin_ground[0])
        msg.pose.position.y = float(origin_ground[1])
        msg.pose.position.z = float(origin_ground[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_ground_pub.publish(msg)

    def _intersect_ray_with_plane(
        self, origin_base: np.ndarray, dir_base: np.ndarray
    ):
        """Intersect ray P(t)=origin+ t*dir with plane a x + b y + c z + d = 0."""
        a, b, c, d = self.plane_coeffs  # numpy array

        num = a * origin_base[0] + b * origin_base[1] + c * origin_base[2] + d
        den = a * dir_base[0] + b * dir_base[1] + c * dir_base[2]

        if abs(den) < 1e-6:
            # Ray is parallel to plane (or nearly)
            return None

        t_param = -float(num) / float(den)
        if t_param < 0.0:
            # Intersection is behind the ray origin
            return None

        P = origin_base + t_param * dir_base
        return P

    def _publish_intersection(self, P_base: np.ndarray, src_msg: Image):
        """Publish intersection as a PointStamped + CYLINDER marker in ground_frame."""
        # PointStamped (for debugging / other nodes)
        pt = PointStamped()
        pt.header.stamp = src_msg.header.stamp
        pt.header.frame_id = self.ground_frame
        pt.point.x = float(P_base[0])
        pt.point.y = float(P_base[1])
        pt.point.z = float(P_base[2])
        self.pt_pub.publish(pt)

        # Cylinder marker aligned with plane normal
        m = Marker()
        m.header = pt.header
        m.ns = "aruco_ground_intersection"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        radius = float(self.intersection_radius)
        height = float(self.intersection_height)

        a, b, c, d = self.plane_coeffs
        n = np.array([a, b, c], dtype=np.float32)
        n_norm = np.linalg.norm(n)

        m.scale.x = 2.0 * radius
        m.scale.y = 2.0 * radius
        m.scale.z = height

        if n_norm < 1e-6:
            # Bad plane normal; just drop a vertical cylinder centered at P
            m.pose.position.x = pt.point.x
            m.pose.position.y = pt.point.y
            m.pose.position.z = pt.point.z
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
        else:
            n_unit = n / n_norm

            # Place cylinder so its base lies on plane and axis along n
            center = P_base + 0.5 * height * n_unit
            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])

            # Build quaternion that rotates +Z to n_unit
            z = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            v = np.cross(z, n_unit)
            cdot = float(np.dot(z, n_unit))
            v_norm = np.linalg.norm(v)

            if v_norm < 1e-6:
                # n_unit is parallel or anti-parallel to +Z
                if cdot > 0.0:
                    qx = 0.0
                    qy = 0.0
                    qz = 0.0
                    qw = 1.0
                else:
                    # 180 deg rotation around X axis
                    qx = 1.0
                    qy = 0.0
                    qz = 0.0
                    qw = 0.0
            else:
                v = v / v_norm
                s = math.sqrt((1.0 + cdot) * 2.0)
                qw = 0.5 * s
                qx = v[0] / s
                qy = v[1] / s
                qz = v[2] / s

            m.pose.orientation.x = float(qx)
            m.pose.orientation.y = float(qy)
            m.pose.orientation.z = float(qz)
            m.pose.orientation.w = float(qw)

        m.color.r = 0.2
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.85

        # persistent marker
        m.lifetime = Duration(seconds=0.0).to_msg()

        self.marker_pub.publish(m)

    @staticmethod
    def _quaternion_to_rotation_matrix(qx, qy, qz, qw):
        """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        R = np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )
        return R

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray):
        tr = R[0, 0] + R[1, 1] + R[2, 2]

        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                qw = (R[2, 1] - R[1, 2]) / S
                qx = 0.25 * S
                qy = (R[0, 1] + R[1, 0]) / S
                qz = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                qw = (R[0, 2] - R[2, 0]) / S
                qx = (R[0, 1] + R[1, 0]) / S
                qy = 0.25 * S
                qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                qw = (R[1, 0] - R[0, 1]) / S
                qx = (R[0, 2] + R[2, 0]) / S
                qy = (R[1, 2] + R[2, 1]) / S
                qz = 0.25 * S

        return float(qx), float(qy), float(qz), float(qw)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()