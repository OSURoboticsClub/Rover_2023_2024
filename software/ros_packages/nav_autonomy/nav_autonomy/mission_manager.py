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

# Custom action and message
from nav_autonomy_interface.action import Mission

from nav_autonomy.utils.search_fsm import SearchFSM, SearchState, SearchPattern


class MissionManager(Node):
    """
    MissionManager Action Server
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        self.navigator = BasicNavigator("basic_navigator")
        self.fromLL_client = self.navigator.create_client(FromLL, '/fromLL')
        self.waypoints: List[PoseStamped] = []  

        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.search_fsm = SearchFSM(
            node=self, 
            navigator=self.navigator, 
            confidence_topic="yolo/confidence", 
            status_topic="search/status"
        )
        
        self.get_logger().info('MissionManager action server ready.')


    def gps_to_map_pose(self, lat, lon, yaw=0.0):           # KRJ TODO: Need to test how nav2 behaves if yaw on each point is 0
        """Use robot_localization to convert GPS to map pose"""

        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = 0.0
        self.get_logger().info('Wait for service')

        # Wait for service
        self.fromLL_client.wait_for_service()
        self.get_logger().info('After service')

        # Call service


        future = self.fromLL_client.call_async(request)
        self.get_logger().info('Before spin')

        rclpy.spin_until_future_complete(self.navigator, future)
        self.get_logger().info('After spin')

        response = future.result()

        # Create PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = response.map_point.x
        pose.pose.position.y = response.map_point.y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


    # -------------------------
    # Action server callbacks
    # -------------------------

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received goal request: '
            f'search_object={goal_request.search_object}, '
            f'search_pattern={goal_request.search_pattern}, '
            f'num_waypoints={len(goal_request.nav_waypoints)}'
        )

        if self.search_fsm.is_active():
            self.get_logger().info('Rejecting new goal - already executing a mission')
            return GoalResponse.REJECT
        
        if len(goal_request.nav_waypoints) == 0:
            self.get_logger().warn('Rejecting goal - no waypoints provided')
            return GoalResponse.REJECT

        # # Validate input
        # if goal_request.search_pattern != Mission.Goal.NO_PATTERN:
        #     if goal_request.search_object not in [Mission.Goal.BOTTLE, Mission.Goal.OG_HAMMER, Mission.Goal.ORANGE_HAMMER]:
        #         self.get_logger().warn('Rejecting goal - invalid or missing search object')
        #         return GoalResponse.REJECT
        self.get_logger().info('Setting waypoints...')

        self.waypoints = [self.gps_to_map_pose(gps.latitude, gps.longitude) for gps in goal_request.nav_waypoints]          # KRJ TODO: Is converting all at once bad if gps corrects?

        self.get_logger().info('Accepting goal request')
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
            self.get_logger().info('Starting mission execution')
            req = goal_handle.request

            # Convert waypoints
            waypoints = self.waypoints
            #for gps in req.nav_waypoints:
            #    wp = await self.gps_to_map_pose(gps.latitude, gps.longitude)
            #    self.get_logger().info(f'Waypoint: lat={gps.latitude}, lon={gps.longitude} -> map=({wp.pose.position.x}, {wp.pose.position.y})')
            #    if wp is None:
            #        self.get_logger().error(f'Failed to convert GPS ({gps.latitude}, {gps.longitude}) to map pose')
            #        goal_handle.abort()
            #        result = Mission.Result()
            #        result.ack = Mission.Result.CANCELLED
            #        return result
            #    waypoints.append(wp)
                

            # ==============================
            # Call Yolo Action Server
            # ==============================
            self.get_logger().info(f'Connecting to YOLO action server.')


            # ==============================
            # Execute search
            # ==============================
            self.get_logger().info(f'Mapping search pattern and starting FSM.')

            search_pattern = SearchPattern.NONE
            match req.search_pattern:
                case Mission.Goal.SPIRAL:
                    search_pattern = SearchPattern.SPIRAL
                case Mission.Goal.LAWNMOWER:
                    search_pattern = SearchPattern.LAWNMOWER
                    # TODO: convert corners to map points
                case _:
                    self.get_logger().error(f'Unknown search pattern: {req.search_pattern}. Falling back to lawnmower.')
                    search_pattern = SearchPattern.LAWNMOWER

            self.get_logger().info(f'Starting search fsm with pattern: {search_pattern.name}')
            self.search_fsm.start(
                start_path=waypoints,
                search_points=waypoints[-1:],
                pattern=search_pattern,
                success_threshold=0.9,
                investigate_threshold=0.7
            )

            # ==============================
            # Poll search feedback
            # ==============================

            while rclpy.ok() and self.search_fsm.is_active():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Mission.Result(ack=Mission.Result.CANCELED)

                goal_handle.publish_feedback(self._map_search_feedback())
                time.sleep(0.4) #TODO: adjust polling rate


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

        elif fsm_state in (SearchState.SEARCHING, SearchState.INVESTIGATING,):
            fb.current_action = Mission.Feedback.SEARCHING
            fb.completion_status = Mission.Feedback.IN_PROGRESS

        elif fsm_state == SearchState.SUCCESS:
            fb.current_action = Mission.Feedback.IDLE
            fb.completion_status = Mission.Feedback.COMPLETED

        elif fsm_state == SearchState.FAILED:
            fb.current_action = Mission.Feedback.IDLE
            fb.completion_status = Mission.Feedback.FAILED

        elif fsm_state == SearchState.STOPPED:
            fb.current_action = Mission.Feedback.NO_ACTION
            fb.completion_status = Mission.Feedback.NOT_STARTED

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
