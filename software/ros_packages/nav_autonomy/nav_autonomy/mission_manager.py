#!/usr/bin/env python3

import asyncio
from typing import List
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from robot_localization.srv import FromLL

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from nav_autonomy_interface.action import Mission, YoloFind
from nav_autonomy.utils.search_fsm import SearchFSM, SearchState, SearchPattern
from nav_autonomy.utils.lights_controller import LightsController


def get_yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        self.lights = LightsController(self)

        self.navigator = BasicNavigator("basic_navigator")
        self.fromLL_client = self.navigator.create_client(FromLL, '/fromLL')

        self.current_pose: PoseStamped = None

        # FIX: These should be parameters in a config or set via goal request.
        self.confidence_threshold_investigate = 0.5 
        self.confidence_threshold_success = 0.70

        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self._yolo_client = ActionClient(
            self,
            YoloFind,
            'YoloFind',
            callback_group=self.callback_group
        )

        self.search_fsm = SearchFSM(
            node=self, 
            navigator=self.navigator, 
            investigate_threshold=self.confidence_threshold_investigate,
            success_threshold=self.confidence_threshold_success,
            status_topic="search/status",
            debug_markers=True
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            '/odometry/global',  
            self._pose_callback,
            10
        )

        self.latest_yolo_feedback = None
        # self.fsm_timer = self.create_timer(0.2, self._fsm_tick)

        self.get_logger().info('MissionManager action server ready.')


    def _pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose


    def _fsm_tick(self):
        self.search_fsm.tick()


    def gps_to_map_pose(self, lat, lon, yaw=0.0):
        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = 0.0

        # Wait for service
        self.fromLL_client.wait_for_service()
        future = self.fromLL_client.call_async(request)
        rclpy.spin_until_future_complete(self.navigator, future)
        response = future.result()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = response.map_point.x
        pose.pose.position.y = response.map_point.y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


    # -------------------------
    # Action server callbacks
    # -------------------------
    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'\n=== New Mission Received ===\n'
            f'Pattern Enum: {goal_request.search_pattern}\n'
            f'Object Enum: {goal_request.search_object}\n'
            f'Param 1 (Spacing): {goal_request.search_param_1:.2f}\n'
            f'Param 2 (Step/Radius): {goal_request.search_param_2:.2f}\n'
            f'Total Waypoints: {len(goal_request.nav_waypoints)}\n'
            f'============================'
        )

        if self.search_fsm.is_active():
            self.get_logger().info('Rejecting new goal - already executing a mission')
            return GoalResponse.REJECT
        if len(goal_request.nav_waypoints) == 0:
            self.get_logger().warn('Rejecting goal - no waypoints provided')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    # Execute mission
    async def execute_callback(self, goal_handle):
        self.yolo_goal_handle = None
        self.yolo_result_future = None

        self.lights.auton_on()

        try:
            req = goal_handle.request

            map_wps = [self.gps_to_map_pose(gps.latitude, gps.longitude) for gps in req.nav_waypoints]

            start_waypoints = []
            search_input_waypoints = []
            search_pattern = SearchPattern.NONE

            if req.search_pattern == Mission.Goal.SPIRAL:
                start_waypoints = map_wps[:-1] # last waypoint is center of spiral
                search_input_waypoints = [map_wps[-1]]
                search_pattern = SearchPattern.SPIRAL
            elif req.search_pattern == Mission.Goal.LAWNMOWER:
                if len(map_wps) >= 4:
                    start_waypoints = map_wps[:-4] # last 4 waypoints are corners of lawnmower
                    search_input_waypoints = map_wps[-4:]
                search_pattern = SearchPattern.LAWNMOWER
            else:
                start_waypoints = map_wps


            # ==============================
            # Call Yolo Action Server
            # ==============================
            if not self._yolo_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('YOLO action server not available')
                goal_handle.abort()
                return Mission.Result(ack=Mission.Result.FAILED)

            yolo_goal = YoloFind.Goal()
            yolo_goal.search_object = req.search_object
            yolo_goal.confidence_threshold = self.confidence_threshold_investigate

            self.get_logger().info('Sending YOLO goal')
            self.yolo_goal_handle = await self._yolo_client.send_goal_async(
                yolo_goal,
                feedback_callback=self._yolo_feedback_cb
            )

            if not self.yolo_goal_handle.accepted:
                self.get_logger().error('YOLO goal rejected')
                goal_handle.abort()
                return Mission.Result(ack=Mission.Result.FAILED)

        
            # ==============================
            # Execute search
            # ==============================
            self.get_logger().info(
                f'Parsed Mission Routing:\n'
                f'  Start Waypoints: {len(start_waypoints)}\n'
                f'  Search Input Waypoints: {len(search_input_waypoints)}\n'
                f'  Mapped Pattern: {search_pattern.name}'
            )

            self.search_fsm.start(
                start_path=start_waypoints,
                search_inputs=search_input_waypoints,
                pattern=search_pattern,
                param1=req.search_param_1,
                param2=req.search_param_2
            )

            # ==============================
            # Poll search feedback
            # ==============================
            while rclpy.ok() and self.search_fsm.is_active():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    if self.yolo_goal_handle:
                        await self.yolo_goal_handle.cancel_goal_async()
                    return Mission.Result(ack=Mission.Result.CANCELED)
                
                if self.latest_yolo_feedback:
                    self.search_fsm.update_perception(self.latest_yolo_feedback.confidence, self.latest_yolo_feedback.pose)
                    self.latest_yolo_feedback = None
                
                self.search_fsm.tick()

                goal_handle.publish_feedback(self._map_search_feedback())
                time.sleep(0.4)

                # Fallback to green even if nav stuck
                if self.search_fsm.get_state() in (SearchState.SUCCESS, SearchState.FAILED):
                    self.lights.success()


            # ==============================
            # Complete action
            # ==============================
            result = Mission.Result()
            final_state = self.search_fsm.get_state()

            if final_state == SearchState.SUCCESS:
                self.get_logger().info('Mission succeeded')
                goal_handle.succeed()
                result.ack = Mission.Result.SUCCESS
                # TODO: Send back detection frame?
            else:
                self.get_logger().info(f'Mission stopped with state: {final_state.name}')
                goal_handle.abort()
                result.ack = Mission.Result.FAILED

            self.lights.success()
            
            return result

        finally:
            self.search_fsm.stop()


    def _yolo_feedback_cb(self, msg):
        fb = msg.feedback
        # no detection, no pose, no care.
        if fb.detected:
            self.latest_yolo_feedback = fb # buffer until next FSM tick


    def _map_search_feedback(self):
        fb = Mission.Feedback()
        fb.header = Header()
        fb.header.stamp = self.get_clock().now().to_msg()

        # Map FSM state to Action feedback
        fsm_state = self.search_fsm.get_state()

        if fsm_state == SearchState.MOVING_TO_START:
            fb.current_action = Mission.Feedback.NAVIGATING
            fb.completion_status = Mission.Feedback.IN_PROGRESS

        elif fsm_state == SearchState.SEARCHING:
            fb.current_action = Mission.Feedback.SEARCHING
            fb.completion_status = Mission.Feedback.IN_PROGRESS

        elif fsm_state == SearchState.INVESTIGATION_PENDING:
            fb.current_action = Mission.Feedback.INVESTIGATING
            fb.completion_status = Mission.Feedback.IN_PROGRESS

        elif fsm_state == SearchState.INVESTIGATING:
            fb.current_action = Mission.Feedback.INVESTIGATING
            fb.completion_status = Mission.Feedback.IN_PROGRESS

        elif fsm_state == SearchState.SUCCESS:
            fb.current_action = Mission.Feedback.NO_ACTION
            fb.completion_status = Mission.Feedback.COMPLETED

        elif fsm_state == SearchState.FAILED:
            fb.current_action = Mission.Feedback.NO_ACTION
            fb.completion_status = Mission.Feedback.FAILED
            
        elif fsm_state == SearchState.STOPPED:
            fb.current_action = Mission.Feedback.NO_ACTION
            fb.completion_status = Mission.Feedback.NOT_STARTED

        if self.current_pose:
            fb.current_x = self.current_pose.pose.position.x
            fb.current_y = self.current_pose.pose.position.y
            yaw_rad = get_yaw_from_quaternion(self.current_pose.pose.orientation)
            fb.current_heading = math.degrees(yaw_rad)
        else:
            fb.current_x = 0.0
            fb.current_y = 0.0
            fb.current_heading = 0.0

        return fb
    

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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

if __name__ == '__main__':
    main()
