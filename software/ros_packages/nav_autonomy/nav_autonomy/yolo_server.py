import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from nav_autonomy_interface.action import YoloFind
from nav_autonomy_interface.srv import SwitchCamera

# TF2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

# YOLO specific
from ultralytics import YOLO
from collections import deque
import numpy as np
from geometry_msgs.msg import Point
import cv2
import torch

torch.backends.cudnn.enabled = False
torch.backends.cuda.matmul.allow_tf32 = True  # use TF32 instead, faster on Jetson                   


class ActionCanceled(Exception):
    """Custom exception to handle action cancellation"""
    pass


class YoloServer(Node):
    """
    Yolo wrapper - Action server for YoloFind
    """

    def __init__(self):
        super().__init__("yolo_server")

        self.callback_group = ReentrantCallbackGroup()

        self.busy = False  # Is a search already being executed?
        self.feedback = None

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsics - will be populated from parameters
        self.left_camera_matrix = None
        self.left_dist_coeffs = None
        self.right_camera_matrix = None
        self.right_dist_coeffs = None
        
        # Get camera intrinsics from camera nodes
        self.get_camera_intrinsics()
   
        # Action Server
        self._action_server = ActionServer(
            self,
            YoloFind,
            "YoloFind",
            execute_callback=self.action_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Service Client: Switch Camera
        self.switch_camera_client = self.create_client(SwitchCamera, f'/rover2_camera/switch_camera')

        self.get_logger().info("YoloServer action server ready.")

        # YOLO PARAMETERS
        self.num_cameras = 1
        self.source = "/dev/rover/camera_infrared"
        self.quit = False
        
        # Camera frame names
        self.left_camera_frame = self.get_parameter("left_camera_frame").value
        self.right_camera_frame = self.get_parameter("right_camera_frame").value
        self.nav_frame = self.get_parameter("nav_frame").value
            
        # Confidence thresholds
        self.stop_threshold = self.get_parameter("stop_threshold").value
        self.detect_threshold = self.get_parameter("detect_threshold").value
        
        # Frame Storage
        self.max_frames = self.get_parameter("max_frames").value
        self.check_frames = self.get_parameter("check_frames").value

    def switch_camera(self, cam_idx):
        if not self.switch_camera_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'switch_camera service for muxing node not available')
            return SwitchCamera.Response.FAIL

        request = SwitchCamera.Request()
        request.cam_idx = cam_idx
        
        future = self.switch_camera_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() == None:
            self.get_logger().warn(f'switch_camera service for muxing node timed out')
            return SwitchCamera.Response.FAIL
        
        return future.result().ack
        
    def get_camera_intrinsics(self):
        """Retrieve camera intrinsics from camera nodes via parameter service"""
        # Get left camera intrinsics
        self.get_logger().info("Requesting left camera intrinsics...")
        left_params = self._get_node_parameters('/rover2_camera/chassis_left_cam', 
                                                  ['camera_matrix', 'distortion_coefficients'])
        if left_params:
            self.left_camera_matrix = np.array(left_params['camera_matrix']).reshape(3, 3)
            self.left_dist_coeffs = np.array(left_params['distortion_coefficients'])
            self.get_logger().info(f"Left camera matrix: {self.left_camera_matrix}")
        
        # Get right camera intrinsics
        self.get_logger().info("Requesting right camera intrinsics...")
        right_params = self._get_node_parameters('/rover2_camera/chassis_right_cam',
                                                   ['camera_matrix', 'distortion_coefficients'])
        if right_params:
            self.right_camera_matrix = np.array(right_params['camera_matrix']).reshape(3, 3)
            self.right_dist_coeffs = np.array(right_params['distortion_coefficients'])
            self.get_logger().info(f"Right camera matrix: {self.right_camera_matrix}")

    def _get_node_parameters(self, node_name, param_names):
        """Helper to get parameters from another node"""
        param_client = self.create_client(GetParameters, f'{node_name}/get_parameters')
        
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Parameter service for {node_name} not available')
            return None
        
        request = GetParameters.Request()
        request.names = param_names
        
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().warn(f'Failed to get parameters from {node_name}')
            return None
        
        response = future.result()
        params = {}
        
        for i, name in enumerate(param_names):
            param_value = response.values[i]
            if param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                params[name] = list(param_value.double_array_value)
            elif param_value.type == ParameterType.PARAMETER_DOUBLE:
                params[name] = param_value.double_value
        
        return params

    def pixel_to_3d_point(self, pixel_x, pixel_y, estimated_depth, camera_id):
        """
        Convert pixel coordinates to 3D point in camera frame
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            estimated_depth: Estimated distance from camera (meters)
            camera_id: 0 for left, 1 for right
        
        Returns:
            Point in camera frame (x, y, z)
        """
        # Select appropriate camera matrix
        if camera_id == 0:
            camera_matrix = self.left_camera_matrix
            dist_coeffs = self.left_dist_coeffs
        else:
            camera_matrix = self.right_camera_matrix
            dist_coeffs = self.right_dist_coeffs
        
        if camera_matrix is None:
            self.get_logger().warn(f"Camera {camera_id} intrinsics not available")
            return None
        
        # Undistort the pixel point
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(pixel_point, camera_matrix, dist_coeffs, P=camera_matrix)
        undist_x, undist_y = undistorted[0][0]
        
        # Get camera intrinsics
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # Convert to normalized camera coordinates
        x_norm = (undist_x - cx) / fx
        y_norm = (undist_y - cy) / fy
        
        # Scale by depth to get 3D point in camera frame
        # Camera frame: X=right, Y=down, Z=forward
        point_camera = np.array([
            x_norm * estimated_depth,  # X (horizontal)
            y_norm * estimated_depth,  # Y (vertical)
            estimated_depth            # Z (depth/forward)
        ])
        
        return point_camera

    def transform_to_nav_frame(self, point_camera, camera_id):
        """
        Transform point from camera frame to base_link frame
        
        Args:
            point_camera: [x, y, z] in camera frame
            camera_id: 0 for left, 1 for right
        
        Returns:
            PoseStamped in base_link frame
        """
        # Select appropriate camera frame
        camera_frame = self.left_camera_frame if camera_id == 0 else self.right_camera_frame
        
        try:
            # Get transform from camera to base_link
            transform = self.tf_buffer.lookup_transform(
                self.nav_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Create PoseStamped in camera frame
            pose_camera = PoseStamped()
            pose_camera.header.frame_id = camera_frame
            pose_camera.pose.position.x = float(point_camera[2])
            pose_camera.pose.position.y = float(-point_camera[1])
            pose_camera.pose.position.z = 0.0
            pose_camera.pose.orientation.w = 1.0

            # Transform to base_link
            pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_camera, transform)
            return pose_base
            
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    def estimate_depth_from_bbox(self, bbox_width, bbox_height, known_object_size=0.3):
        """
        Estimate depth based on bounding box size
        This is a simple heuristic - adjust based on your objects
        
        Args:
            bbox_width, bbox_height: Bounding box dimensions in pixels
            known_object_size: Approximate real-world size of object (meters)
        
        Returns:
            Estimated depth in meters
        """
        # Use the larger dimension
        bbox_size = max(bbox_width, bbox_height)
        
        # Simple inverse relationship (tune this based on testing)
        # focal_length can be approximated from camera matrix
        focal_length = (self.left_camera_matrix[0, 0] + self.left_camera_matrix[1, 1]) / 2
        
        estimated_depth = ((known_object_size * focal_length) / bbox_size)/2
        
        return estimated_depth
    
    def pose_marker_logging(self, pose):
        self.get_logger().info(
            f"Object position in map: "
            f"x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}, "
            f"z={pose.pose.position.z:.2f}"
        )
        marker = Marker()
        marker.header.frame_id = self.nav_frame  # "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "yolo"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = pose.pose

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def calc_detection_point(self, cam_idx, bbox_dims, xc, yc):
        # Estimate depth from bounding box size
        estimated_depth = self.estimate_depth_from_bbox(
            bbox_dims[0], 
            bbox_dims[1],
            known_object_size=0.3  # TODO: Adjust based on your object
        )

        self.get_logger().info(f"Estimated depth: {estimated_depth:.2f}m")
        
        # Convert pixel to 3D point in camera frame
        point_camera = self.pixel_to_3d_point(
            xc, 
            yc, 
            estimated_depth,
            cam_idx
        )
        
        if point_camera is not None:
            self.get_logger().info(f"Point in camera frame: {point_camera}")

            # Transform to base_link frame
            pose_base = self.transform_to_nav_frame(
                point_camera,
                cam_idx
            )
            if pose_base is not None:
                self.pose_marker_logging(pose_base)
                self.target_pose_pub.publish(pose_base)
                return pose_base
            
        return None

    def _draw_detections(self, frame, boxes_xyxy, conf_scores, best_idx):
        """
        Draw bounding boxes on a frame.
        All detections drawn in green, best detection highlighted in red.
        Returns the annotated frame.
        """
        annotated = frame.copy()

        # Draw all detections in green
        for i, (box, conf) in enumerate(zip(boxes_xyxy, conf_scores)):
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # Highlight the best detection in red
        x1, y1, x2, y2 = map(int, boxes_xyxy[best_idx])
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(
            annotated,
            f"BEST {conf_scores[best_idx]:.2f}",
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )

        return annotated


    async def action_callback(self, goal_handle):
        class ActionCanceled(Exception):
            """Custom exception to handle action cancellation"""
            pass

        self.xc = None
        self.yc = None
        
        self.bbox_dims = None
        
        cap = None

        try:
            goal = goal_handle.request
            model = YOLO("yolo_models/mallet.pt")
            
            # Define model from action request
            if goal.search_object == YoloFind.Goal.BOTTLE:
                model = YOLO("yolo_models/bottle.pt")
            elif goal.search_object == YoloFind.Goal.ORANGE_HAMMER:
                model = YOLO("yolo_models/mallet.pt")
            elif goal.search_object == YoloFind.Goal.OG_HAMMER:
                model = YOLO("yolo_models/hammer.pt")
            model.overrides['verbose'] = False

            # Run Yolo or Aruco
            if goal.search_object == YoloFind.Goal.ARUCO:
                await self.do_aruco(goal_handle)
                return
            else:
                cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)

                camera_stacks = [deque(maxlen=self.max_frames) for _ in range(self.num_cameras)]
                # Start searching in camera stream for object(s)
                
                frame_id = 0
                cam_idx = 0

                while cap.isOpened():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.busy = False
    #                    cv2.destroyAllWindows()
                        cap.release()
                        return YoloFind.Result()

                    # Switch Camera
                    while True:
                        cam_idx = (cam_idx + 1) % self.num_cameras
                        if self.switch_camera(cam_idx) == SwitchCamera.Response.SUCCESS:
                            break
                        self.get_logger().warn("Failed to switch cameras")
                        camera_stacks[cam_idx].append(0.0)

                    # Get Frame
                    ret, frame = cap.read()
                    if not ret:
                        self.get_logger().warn("Failed to read frame")
                        continue

                    # Run YOLO on frame
                    result = model(frame, device=0)[0]

                    display_frame = frame.copy()

                    # Check if detection in frame
                    if result.boxes is None or len(result.boxes) == 0:
                        # No detection
                        camera_stacks[cam_idx].append(0.0)
                    else:
                        # Get best detection from the frame
                        boxes_xyxy = result.boxes.xyxy.tolist()
                        conf_scores = result.boxes.conf.tolist()
                        best_idx = np.argmax(conf_scores)
                        current_conf = conf_scores[best_idx]

                        # Draw bounding boxes on display frame
                        display_frame = self._draw_detections(
                            frame, boxes_xyxy, conf_scores, best_idx
                        )
                        
                        camera_stacks[cam_idx].append(current_conf)

                    # Grab average of list and check against thresholds
                    total_mean = sum(camera_stacks[cam_idx]) / len(camera_stacks[cam_idx])

                    # Check if this camera has better detection
                    if self.best_cam_idx == cam_idx or total_mean > self.best_mean_conf:
                        self.best_cam_idx = cam_idx
                        self.best_mean_conf = total_mean

                        # recent_stack = camera_stacks[current_cam][-self.check_frames :]:
                        # recent_mean = sum(recent_stack) / self.check_frames

                        # Overlay stats on display frame
#                        cv2.putText(
#                            display_frame,
#                            f"Total Mean: {total_mean:.2f}  Recent Mean: {recent_mean:.2f}",
#                            (10, 30),
#                            cv2.FONT_HERSHEY_SIMPLEX,
#                            0.6,
#                            (255, 255, 0),
#                            2,
#                        )

                        if total_mean >= self.stop_threshold:
                            # Show final frame before breaking
 #                           cv2.imshow("YOLO Detection", display_frame)
 #                           cv2.waitKey(1)
                            break

                        center, top_left, bottom_right = self.get_points()
                        feedback = YoloFind.Feedback()
                        feedback.confidence = current_conf
                        feedback.frame_id = frame_id
                        feedback.detected = True
                        feedback.total_conf = total_mean
                        feedback.center = center
                        feedback.top_left = top_left
                        feedback.bottom_right = bottom_right
                        feedback.pose = PoseStamped()

                        if self.xc and self.yc:
                            # Estimate depth from bounding box size
                            estimated_depth = self.estimate_depth_from_bbox(
                                self.bbox_dims[0], 
                                self.bbox_dims[1],
                                known_object_size=0.3  # Adjust based on your object
                            )
                            
                            self.get_logger().info(f"Estimated depth: {estimated_depth:.2f}m")
                            
                            # Convert pixel to 3D point in camera frame
                            point_camera = self.pixel_to_3d_point(
                                self.xc, 
                                self.yc, 
                                estimated_depth,
                                cam_idx
                            )
                            
                            if point_camera is not None:
                                self.get_logger().info(f"Point in camera frame: {point_camera}")
                                
                                # Transform to base_link frame
                                pose_base = self.transform_to_base_frame(
                                    point_camera,
                                    cam_idx
                                )
                                
                                if pose_base is not None:
                                    self.get_logger().info(
                                        f"Object position in map: "
                                        f"x={pose_base.pose.position.x:.2f}, "
                                        f"y={pose_base.pose.position.y:.2f}, "
                                        f"z={pose_base.pose.position.z:.2f}"
                                    )
                                    marker = Marker()
                                    marker.header.frame_id = self.base_frame  # "map"
                                    marker.header.stamp = self.get_clock().now().to_msg()

                                    marker.ns = "yolo"
                                    marker.id = 0
                                    marker.type = Marker.SPHERE
                                    marker.action = Marker.ADD

                                    marker.pose = pose_base.pose

                                    marker.scale.x = 0.2
                                    marker.scale.y = 0.2
                                    marker.scale.z = 0.2

                                    marker.color.r = 1.0
                                    marker.color.g = 0.0
                                    marker.color.b = 0.0
                                    marker.color.a = 1.0

                                    self.marker_pub.publish(marker)
                                    self.target_pose_pub.publish(pose_base)
                                    feedback.pose = pose_base

                        goal_handle.publish_feedback(feedback)

                    else:
                        # No detections - publish feedback with no detection
                        feedback = YoloFind.Feedback()
                        feedback.confidence = 0.0
                        feedback.frame_id = frame_id
                        feedback.detected = False
                        goal_handle.publish_feedback(feedback)

                    # Display the frame (with or without detections)
  #                  cv2.imshow("YOLO Detection", display_frame)

                    # Press 'q' to manually quit the window
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                    frame_id += 1

                cap.release()
   #             cv2.destroyAllWindows()

                # ==============================
                # Calculate 3D position
                # ==============================
                    
                        
        except ActionCanceled:
            self.get_logger().info("Yolo search canceled")
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.CANCELED
            self.busy = False
            return result
        except Exception as e:
            self.get_logger().error(f"Action callback error: {e}")
            self.busy = False
            if cap is not None:
                cap.release()
            cv2.destroyAllWindows()
            raise

    def get_points(self):
        center = Point()
        center.x = xc
        center.y = yc
        center.z = 0.0

        x1, y1, x2, y2 = best_boxes

        top_left = Point()
        top_left.x = x1
        top_left.y = y1
        top_left.z = 0.0

        bottom_right = Point()
        bottom_right.x = x2
        bottom_right.y = y2
        bottom_right.z = 0.0
        
        return center, top_left, bottom_right

    def do_aruco(self, goal_handle):
        self.get_logger().info("detecting '{}' tags...".format(cv2.aruco.DICT_4X4_50))
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        
        # Create the detector with dictionary and parameters
        detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
        
        cap = cv2.VideoCapture(self.source)

        camera_matrix = np.array(
            [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]
        )
        dist_coeffs = np.zeros((5, 1))

        marker_length = 0.2  # meters

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    raise ActionCanceled()
                
                ret, frame = cap.read()
                if not ret:
                    break
                
                display_frame = frame.copy()
                
                # Detect markers
                (corners, ids, rejected) = detector.detectMarkers(frame)
                
                if ids is not None:
                    cv2.aruco.drawDetectorMarkers(display_frame, corners, ids)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, marker_length, camera_matrix, dist_coeffs
                    )
                    tvec = tvecs[0][0]

                    # Overlay pose info on frame
                    cv2.putText(
                        display_frame,
                        f"X:{tvec[0]:.2f} Y:{tvec[1]:.2f} Z:{tvec[2]:.2f}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

                    feedback = YoloFind.Feedback()
                    feedback.detected = True
                    feedback.pose.position.x = float(tvec[0])
                    feedback.pose.position.y = float(tvec[1])
                    feedback.pose.position.z = float(tvec[2])
                    goal_handle.publish_feedback(feedback)

                else:
                    feedback = YoloFind.Feedback()
                    feedback.detected = False
                    goal_handle.publish_feedback(feedback)
                
                cv2.imshow("ArUco Detection", display_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.FAILED
            return result

        finally:
            cap.release()
            cv2.destroyAllWindows()

    def do_yolo(self, goal_handle):
        cap = None
        best_cam_idx = -1
        best_mean_conf = 0.0
        frame_id = 0
        cam_idx = 0

        feedback = YoloFind.Feedback()
        feedback.detected = 0.0
        feedback.confidence = 0.0

        try:
            # Define model from action request
            model = YOLO("yolo_models/mallet.pt")
            if goal_handle.request.search_object == YoloFind.Goal.BOTTLE:
                model = YOLO("yolo_models/bottle.pt")
            elif goal_handle.request.search_object == YoloFind.Goal.ORANGE_HAMMER:
                model = YOLO("yolo_models/mallet.pt")
            elif goal_handle.request.search_object == YoloFind.Goal.OG_HAMMER:
                model = YOLO("yolo_models/hammer.pt")
            model.overrides['verbose'] = False

            cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)

            camera_stacks = [deque(maxlen=self.max_frames) for _ in range(self.num_cameras)]

            # Start searching in camera stream for object(s)
            while cap.isOpened():
                if goal_handle.is_cancel_requested:
                    raise ActionCanceled()
                    
                # Switch Camera
                while True:
                    cam_idx = (cam_idx + 1) % self.num_cameras
                    if self.switch_camera(cam_idx) == SwitchCamera.Response.SUCCESS:
                        break
                    self.get_logger().warn("Failed to switch cameras")
                    camera_stacks[cam_idx].append(0.0)

                # Get Frame
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to read frame")
                    continue

                # Run YOLO on frame
                result = model(frame, device=0)[0]
                display_frame = frame.copy()

                # Check for detection
                detected_this_frame = result.boxes is not None and len(result.boxes) > 0
                if detected_this_frame:
                    # Get best detection from the frame
                    boxes_xyxy = result.boxes.xyxy.tolist()
                    conf_scores = result.boxes.conf.tolist()

                    best_idx = np.argmax(conf_scores)
                    current_conf = conf_scores[best_idx]
                    best_boxes = boxes_xyxy[best_idx]
                    xc, yc, w, h = result.boxes.xywh[best_idx].tolist()
                    bbox_dims = (w, h)

                    # Draw bounding boxes on display frame
                    display_frame = self._draw_detections(
                        frame, boxes_xyxy, conf_scores, best_idx
                    )

                else:
                    current_conf = 0.0
                    
                # Calculate the average of recent detections
                camera_stacks[cam_idx].append(current_conf)
                total_mean = sum(camera_stacks[cam_idx]) / len(camera_stacks[cam_idx])

                # Keep track of which camera has best detection
                if total_mean > best_mean_conf or best_cam_idx == cam_idx:
                    best_cam_idx = cam_idx
                    best_mean_conf = total_mean

                # Update feedback if best detection is this frame
                if detected_this_frame and best_cam_idx == cam_idx:
                    center, top_left, bottom_right = self.get_points(best_boxes, xc, yc)
                    feedback.confidence = current_conf
                    feedback.frame_id = frame_id
                    feedback.detected = total_mean > self.detect_threshold
                    feedback.total_conf = total_mean
                    feedback.center = center
                    feedback.top_left = top_left
                    feedback.bottom_right = bottom_right
                    feedback.pose = PoseStamped()

                    # Calculate navigation waypoint pose
                    if feedback.detected:
                        waypoint = self.calc_detection_point(cam_idx, bbox_dims, xc, yc)
                        if waypoint:
                            feedback.pose = waypoint

                goal_handle.publish_feedback(feedback)

                # Display the frame (with or without detections)
                cv2.imshow("YOLO Detection", display_frame)

                # Press 'q' to manually quit the window
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                frame_id += 1

        except ActionCanceled:
            raise
        except Exception as e:
            self.get_logger().error(f"Action callback error: {e}")
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.FAILED
            return result
              
        finally:   
            if cap is not None:
                cap.release()
            cv2.destroyAllWindows()  

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f"Received goal request: search_object={goal_request.search_object}, "
        )
        # Check if already busy
        if self.busy:
            self.get_logger().info("Rejecting new goal - already executing a search")
            return GoalResponse.REJECT

        # Validate input
        if goal_request.search_object not in [
            YoloFind.Goal.BOTTLE,
            YoloFind.Goal.OG_HAMMER,
            YoloFind.Goal.ORANGE_HAMMER,
            YoloFind.Goal.ARUCO
        ]:
            self.get_logger().warn("Rejecting goal - invalid or missing search object")
            return GoalResponse.REJECT

        self.get_logger().info("Accepting goal request")
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def action_callback(self, goal_handle):
        try:
            # Run Yolo or Aruco
            if goal_handle.request.search_object == YoloFind.Goal.ARUCO:
                return self.do_aruco(goal_handle)
            else:
                return self.do_yolo(goal_handle)
                        
        except ActionCanceled:
            self.get_logger().info("Yolo search canceled")
            goal_handle.canceled()
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.CANCELED
            return result
        
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)

    node = YoloServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()