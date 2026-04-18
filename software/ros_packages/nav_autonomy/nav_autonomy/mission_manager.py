#!/usr/bin/env python3

import asyncio
from typing import List
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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

def gps_translation(lat1, lon1, lat2, lon2):
    """
    Calculate the x, y translation (in meters) between two GPS points, assuming a small distance.
    """
    DEGREE_TO_METERS = 111000  # meters per degree of latitude

    lat_avg_rad = (math.radians(lat1) + math.radians(lat2)) / 2

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    dy = dlat * DEGREE_TO_METERS
    dx = dlon * DEGREE_TO_METERS * math.cos(lat_avg_rad)        # Scale dlon based on location

    return dx, dy

def get_yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        self.navigator = BasicNavigator("basic_navigator")
        self.fromLL_client = self.navigator.create_client(FromLL, '/fromLL')

        self.current_pose: PoseStamped = None

        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # self._yolo_client = ActionClient(
        #     self,
        #     YoloFind,
        #     'YoloFind',
        #     callback_group=self.callback_group
        # )

        self.search_fsm = SearchFSM(
            node=self, 
            navigator=self.navigator, 
            confidence_topic="yolo/confidence", 
            status_topic="search/status",
            target_pose_topic="/visualization_marker",
            debug_markers=True
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            '/odometry/global',  
            self._pose_callback,
            10
        )

        self.fsm_timer = self.create_timer(0.2, self._fsm_tick)

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

        # Sanity check for how far lat long is
        sanity_x, sanity_y = gps_translation(self.latest_gps_lat, self.latest_gps_long, lat, lon)
        self.get_logger().info(f'Sanity Check:')
        self.get_logger().info(f'Waypoint should be about this far away dx={sanity_x} dy={sanity_y}')
        self.get_logger().info(f'Current map pose is x={self.latest_map_pose.pose.position.x} y={self.latest_map_pose.pose.position.y}')
        self.get_logger().info(f'Translated waypoint map pose is x={pose.pose.position.x} y={pose.pose.position.y}')

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

    # ------------------------------------------------
    # Execute a mission
    # ------------------------------------------------
    async def execute_callback(self, goal_handle):
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
            # self._send_yolo_action(goal_handle)
        
        
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
                    return Mission.Result(ack=Mission.Result.CANCELED)

                goal_handle.publish_feedback(self._map_search_feedback())
                time.sleep(0.4)

            # ==============================
            # Complete action
            # ==============================
            result = Mission.Result()
            final_state = self.search_fsm.get_state()

            if final_state == SearchState.SUCCESS:
                self.get_logger().info('Mission succeeded')
                goal_handle.succeed()
                result.ack = Mission.Result.SUCCESS
            else:
                self.get_logger().info(f'Mission stopped with state: {final_state.name}')
                goal_handle.abort()
                result.ack = Mission.Result.FAILED

            return result

        finally:
            self.search_fsm.stop()


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


    # async def _send_yolo_action(self, goal_handle):
    #     self.get_logger().info(f'Connecting to YOLO action server.')

    #     if not self._yolo_client.wait_for_server(timeout_sec=5.0):
    #         self.get_logger().error('YOLO action server not available')
    #         goal_handle.abort()
    #         return Mission.Result(ack=Mission.Result.CANCELED)

    #     # Create and send YOLO goal
    #     yolo_goal = YoloFind.Goal()
    #     yolo_goal.search_object = 3  # or req.search_object if passed in Mission.Goal

    #     self.get_logger().info(f'Sending YOLO goal: {yolo_goal.search_object}')
    #     yolo_goal_future = await self._yolo_client.send_goal_async(yolo_goal)

    #     if not yolo_goal_future.accepted:
    #         self.get_logger().error('YOLO goal rejected')
    #         goal_handle.abort()
    #         return Mission.Result(ack=Mission.Result.FAILED)

    #     self.get_logger().info('YOLO goal accepted, waiting for result...')
    #     yolo_result = await yolo_goal_future.get_result_async()

    #     if yolo_result.result.ack != YoloFind.Result.SUCCESS:
    #         self.get_logger().warn(f'YOLO search failed or canceled: {yolo_result.result.ack}')
    #         goal_handle.abort()
    #         return Mission.Result(ack=Mission.Result.FAILED)

    #     self.get_logger().info(
    #         f'YOLO found object at: ({yolo_result.result.center.x:.2f}, {yolo_result.result.center.y:.2f})'
    #     )


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
