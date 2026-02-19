#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header

from nav_autonomy_interface.action import YoloFind

# YOLO specific
from ultralytics import YOLO
from collections import deque
import numpy as np
from geometry_msgs.msg import Point
import time

# ARUCO
import cv2
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

        # YOLO PARAMETERS
        self.num_cameras = 1
        self.source = "/dev/rover/camera_infrared"
        self.quit = False
        self.cam_queue = deque()
        # Append camera carousel order
        self.cam_queue.append(0)
        # Confidence thresholds
        self.stop_threshold = 0.85
        self.check_threshold = 0.60
        # Frame Storage
        self.max_frames = 25
        self.check_frames = 10

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

    async def action_callback(self, goal_handle):
        class ActionCanceled(Exception):
            """Custom exception to handle action cancellation"""

            pass

        try:
            # ==============================
            # Look for object
            # ==============================
            goal = goal_handle.request
            model = YOLO("yolo_models/mallet.pt")
            # Define models from action request
            if goal.search_object == YoloFind.Goal.BOTTLE:
                model = YOLO("yolo_models/bottle.pt")
            elif goal.search_object == YoloFind.Goal.ORANGE_HAMMER:
                model = YOLO("yolo_models/mallet.pt")
            elif goal.search_object == YoloFind.Goal.OG_HAMMER:
                model = YOLO("yolo_models/hammer.pt")
            if goal.search_object == "ARUCO":
                await self.do_aruco(goal_handle)
                return
            else:
                # results = model(source=self.source, stream=True, _backend=cv2.CAP_V4L2)
                cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)

                camera_stacks = [[] for _ in range(self.num_cameras)]
                # Start searching in camera stream for object(s)
                current_cam = self.cam_queue.popleft()
                self.cam_queue.append(current_cam)
                frame_id = 0
                found = False
                while cap.isOpened():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.busy = False
                        return YoloFind.Result()

                    ret, frame = cap.read()
                    if not ret:
                        self.get_logger().warn("Failed to read frame")
                        break

                    # Start time recording
                    start_time = time.perf_counter()

                    # Run YOLO on frame
                    result = model.predict(source=frame, device=0)[
                        0
                    ]  # Process frame through YOLO
                    if (
                        result.boxes is not None and len(result.boxes) > 0
                    ):  # If any object is detected
                        # Grab data from detection
                        boxes_xyxy = (
                            result.boxes.xyxy.tolist()
                        )  # Points from drawn boxes
                        conf_scores = (
                            result.boxes.conf.tolist()
                        )  # All confidence scores from any detected objects
                        best_idx = np.argmax(conf_scores)
                        current_conf = conf_scores[
                            best_idx
                        ]  # Currently detected object

                        # Store obj conf from 50 frames from cam
                        if len(camera_stacks[current_cam]) >= self.max_frames:
                            camera_stacks[current_cam].pop(0)  # remove oldest
                        camera_stacks[current_cam].append(current_conf)  # add newest
                        # Grab average of list and check against
                        self.total_mean = sum(camera_stacks[current_cam]) / len(
                            camera_stacks[current_cam]
                        )
                        recent_stack = camera_stacks[current_cam][-self.check_frames :]
                        self.recent_mean = (
                            sum(recent_stack) / len(recent_stack)
                        )  # Grab the most recent self.check_frames number of frames and average conf

                        self.best_boxes = boxes_xyxy[
                            best_idx
                        ]  # Get box data from detection
                        self.xc, self.yc, _, _ = result.boxes.xywh[
                            best_idx
                        ].tolist()  # Grab points from box data

                        self.center, self.top_left, self.bottom_right = self.plot_point(
                            self.xc, self.yc, self.best_boxes
                        )  # Grab the location of the points

                        feedback = YoloFind.Feedback()  # Send feedback
                        feedback.confidence = current_conf
                        feedback.frame_id = frame_id
                        feedback.detected = False
                        feedback.total_conf = self.total_mean
                        feedback.recent_conf = self.recent_mean
                        feedback.center = self.center
                        feedback.top_left = self.top_left
                        feedback.bottom_right = self.bottom_right
                        goal_handle.publish_feedback(feedback)

                    stop_time = time.perf_counter()
                    elapsed = stop_time - start_time
                    with open("yolo_time_logs.txt", "a") as f:
                        f.write(f"Frame ID: {frame_id} | Time taken: {elapsed}")
                    frame_id += 1
                # TODO: Intermittently send feedback
                cap.release()
                # ==============================
                # Complete action
                # ==============================
                # Bounding boxes - Center
                if not self.quit and found:
                    self.center, self.top_left, self.bottom_right = self.plot_point(
                        self.xc, self.yc, self.best_boxes
                    )

                    self.get_logger().info("Yolo search Completed")
                    goal_handle.succeed()
                    result = YoloFind.Result()
                    result.header = Header()
                    result.ack = YoloFind.Result.SUCCESS
                    result.center = self.center
                    result.top_left = self.top_left
                    result.bottom_right = self.bottom_right
                    self.busy = False
                    return result
        except ActionCanceled:
            # Handle cancellation
            self.get_logger().info("Yolo search canceled")
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.CANCELED
            self.busy = False
            return result

    async def plot_point(self, xc, yc, best_boxes):
        center = Point()
        center.x = xc
        center.y = yc
        center.z = 0.0
        # Bounding boxes - Top left
        x1, y1, x2, y2 = best_boxes
        top_left = Point()
        top_left.x = x1
        top_left.y = y1
        top_left.z = 0.0

        # Bounding boxes - Top left
        bottom_right = Point()
        bottom_right.x = x2
        bottom_right.y = y2
        bottom_right.z = 0.0

        return (center, top_left, bottom_right)

    async def do_aruco(self, goal_handle):
        print("[INFO] detecting '{}' tags...".format(self.ARUCO_size))
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        cap = cv2.VideoCapture(self.source)

        camera_matrix = np.array(
            [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]
        )
        dist_coeffs = np.zeros((5, 1))

        marker_length = 0.2  # meters

        while rclpy.ok() and not goal_handle.is_cancel_requested:
            ret, frame = cap.read()
            if not ret:
                break
            (corners, ids, rejected) = cv2.aruco.detectMarkers(
                frame, arucoDict, parameters=arucoParams
            )

            if ids is not None:
                cv2.aruco.drawDetectorMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_length, camera_matrix, dist_coeffs
                )
                tvec = tvecs[0][0]
                # rvec = rvecs[0][0]

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

        await asyncio.sleep(0.01)

        cap.release()
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
