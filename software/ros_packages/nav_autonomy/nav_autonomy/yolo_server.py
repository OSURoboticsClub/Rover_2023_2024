import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor



from std_msgs.msg import Header

from nav_autonomy_interface.action import YoloFind



# YOLO specific
from ultralytics import YOLO
from collections import deque
import numpy as np
from geometry_msgs.msg import Point
import cv2
import torch

torch.backends.cudnn.enabled = False
# CUDNN still broken - may need to figure out for accel - benchmark difference on home pc w/ nvidia card?

# ARUCO
import asyncio


class YoloServer(Node):
    """
    Yolo wrapper - Action server for YoloFind
    """

    def __init__(self):
        super().__init__("yolo_server")

        self.callback_group = ReentrantCallbackGroup()

        self.busy = False  # Is a search already being executed?

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

        self.get_logger().info("YoloServer action server ready.")

        # TODO: Parameterize these as ROS2 params
        # ROS2 Parameters
        self.declare_parameter(
            "num_cameras",
            1,
            ParameterDescriptor(
                description="Number of cameras carouselling through MUXing node"
            ),
        )
        self.declare_parameter(
            "source",
            "/dev/video64",
            ParameterDescriptor(
                description="Camera source, default to normal camera - change to muxing node"
            ),
        )
        self.declare_parameter(
            "stop_threshold",
            0.85,
            ParameterDescriptor(
                description="Threshold to search entire camera stack for to stop server and send back result"
            ),
        )
        self.declare_parameter(
            "check_threshold",
            0.60,
            ParameterDescriptor(
                description="Threshold to start search pattern to check for object in environment"
            ),
        )
        self.declare_parameter(
            "max_frames",
            25,
            ParameterDescriptor(
                description="Number of frames stored for history per camera, full history checked for stop_threshold"
            ),
        )
        self.declare_parameter(
            "check_frames",
            10,
            ParameterDescriptor(
                description="Number of frames checked in history for check_threshold"
            ),
        )
        self.declare_parameter(
            "camera_carousel",
            [0],
            ParameterDescriptor(description="Array of camera numbers for input"),
        )

        # Internal Parameters
        self.num_cameras = self.get_parameter("num_cameras").value
        self.source = self.get_parameter("source").value
        self.quit = False
        # Append camera carousel order
        self.cam_queue = deque()
        for cam in self.get_parameter("camera_carousel").value:
            self.cam_queue.append(cam)
        # Confidence thresholds
        self.stop_threshold = self.get_parameter("stop_threshold").value
        self.check_threshold = self.get_parameter("check_threshold").value
        # Frame Storage
        self.max_frames = self.get_parameter("max_frames").value
        self.check_frames = self.get_parameter("check_frames").value
       
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
        self.quit = True
        return CancelResponse.ACCEPT

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
        
        cap = None  # Declare here so except block can access it


        try:
            # ==============================
            # Look for object
            # ==============================
            goal = goal_handle.request
            model = YOLO("yolo_models/mallet.pt")

            # Define model from action request
            if goal.search_object == YoloFind.Goal.BOTTLE:
                model = YOLO("yolo_models/bottle.pt")
            elif goal.search_object == YoloFind.Goal.ORANGE_HAMMER:
                model = YOLO("yolo_models/mallet.pt")
            elif goal.search_object == YoloFind.Goal.OG_HAMMER:
                model = YOLO("yolo_models/hammer.pt")

            if goal.search_object == YoloFind.Goal.ARUCO:
                await self.do_aruco(goal_handle)
                return
            else:
                cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)

                camera_stacks = [deque(maxlen=self.max_frames) for _ in range(self.num_cameras)]
               
                # Start searching in camera stream for object(s)
                current_cam = self.cam_queue.popleft()
                self.cam_queue.append(current_cam)
                frame_id = 0

                while cap.isOpened():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.busy = False
                        cv2.destroyAllWindows()
                        cap.release()
                        return YoloFind.Result()

                    ret, frame = cap.read()
                    if not ret:
                        self.get_logger().warn("Failed to read frame")
                        # break

                    # Run YOLO on frame
                    result = model(frame, device=0)[0]

                    # Default display frame (no detections)
                    display_frame = frame.copy()

                    if result.boxes is not None and len(result.boxes) > 0:
                        boxes_xyxy = result.boxes.xyxy.tolist()
                        conf_scores = result.boxes.conf.tolist()
                        best_idx = np.argmax(conf_scores)
                        current_conf = conf_scores[best_idx]

                        # Draw bounding boxes on display frame
                        display_frame = self._draw_detections(
                            frame, boxes_xyxy, conf_scores, best_idx
                        )
                        
                        camera_stacks[current_cam].append(current_conf)


                        # Grab average of list and check against thresholds
                        total_mean = sum(camera_stacks[current_cam]) / self.max_frames

                        # recent_stack = camera_stacks[current_cam][-self.check_frames :]:
                        # recent_mean = sum(recent_stack) / self.check_frames

                        # # Overlay stats on display frame
                        # cv2.putText(
                        #     display_frame,
                        #     f"Total Mean: {total_mean:.2f}  Recent Mean: {recent_mean:.2f}",
                        #     (10, 30),
                        #     cv2.FONT_HERSHEY_SIMPLEX,
                        #     0.6,
                        #     (255, 255, 0),
                        #     2,
                        # )
                        # Lock in best detection
                        self.best_boxes = boxes_xyxy[best_idx]
                        self.xc, self.yc, _, _ = result.boxes.xywh[
                            best_idx
                        ].tolist()

                        if total_mean >= self.stop_threshold:
                            # Show final frame before breaking
                            cv2.imshow("YOLO Detection", display_frame)
                            cv2.waitKey(1)
                            break


                        center, top_left, bottom_right = self.get_points()
                        feedback = YoloFind.Feedback()
                        feedback.confidence = current_conf
                        feedback.frame_id = frame_id
                        feedback.detected = True
                        feedback.total_conf = total_mean
                        # feedback.recent_conf = recent_mean
                        feedback.center = center
                        feedback.top_left = top_left
                        feedback.bottom_right = bottom_right
                        goal_handle.publish_feedback(feedback)


                    else:
                        # No detections - publish feedback with no detection
                        feedback = YoloFind.Feedback()
                        feedback.confidence = 0.0
                        feedback.frame_id = frame_id
                        feedback.detected = False
                        goal_handle.publish_feedback(feedback)

                    # Display the frame (with or without detections)
                    cv2.imshow("YOLO Detection", display_frame)

                    # Press 'q' to manually quit the window
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                    frame_id += 1

                cap.release()
                cv2.destroyAllWindows()

                # ==============================
                # Complete action
                # ==============================
                if not self.quit:
                    if self.xc and self.yc:
                        center, top_left, bottom_right = self.get_points() 

                        self.get_logger().info("Yolo search completed")
                        goal_handle.succeed()
                        result = YoloFind.Result()
                        result.header = Header()
                        result.ack = YoloFind.Result.SUCCESS
                        result.center = center
                        result.top_left = top_left
                        result.bottom_right = bottom_right
                        self.busy = False
                        return result
                    else:
                        return None
        except ActionCanceled:
            self.get_logger().info("Yolo search canceled")
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.CANCELED
            self.busy = False
            return result


    def get_points(self):
        center = Point()
        center.x = self.xc
        center.y = self.yc
        center.z = 0.0

        x1, y1, x2, y2 = self.best_boxes

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
        print("[INFO] detecting '{}' tags...".format(cv2.aruco.DICT_4X4_50))
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
        
        # Define 3D object points for the ArUco marker (in marker coordinate system)
        # Marker corners in 3D space (square centered at origin)
        half_size = marker_length / 2.0
        object_points = np.array([
            [-half_size,  half_size, 0],  # Top-left
            [ half_size,  half_size, 0],  # Top-right
            [ half_size, -half_size, 0],  # Bottom-right
            [-half_size, -half_size, 0]   # Bottom-left
        ], dtype=np.float32)

        while rclpy.ok() and not goal_handle.is_cancel_requested:
            ret, frame = cap.read()
            if not ret:
                break
            
            display_frame = frame.copy()
            
            # Detect markers
            (corners, ids, rejected) = detector.detectMarkers(frame)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                
                # Process the first detected marker
                # corners[0] is the first marker, shape: (1, 4, 2)
                # Reshape to (4, 2) for solvePnP
                image_points = corners[0].reshape((4, 2))
                
                # Estimate pose using solvePnP
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE  # Good for square markers
                )
                
                if success:
                    # Draw axis on the marker (optional, for visualization)
                    cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, 
                                    rvec, tvec, marker_length * 0.5)
                    
                    # Extract position from tvec
                    tvec = tvec.flatten()  # Convert from (3,1) to (3,)
                    
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
            else:
                feedback = YoloFind.Feedback()
                feedback.detected = False
                goal_handle.publish_feedback(feedback)
            
            cv2.imshow("ArUco Detection", display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            
        
        cap.release()
        cv2.destroyAllWindows()
        goal_handle.canceled()
        result = YoloFind.Result()
        result.ack = YoloFind.Result.CANCELED
        return result


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
