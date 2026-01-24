#!/usr/bin/env python3

from typing import List
import time
import math

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
from nav_autonomy_interface.msg import GPSWaypoint

from nav_autonomy.utils import search_patterns

class MissionManager(Node):
    """
    MissionManager Action Server
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator("basic_navigator")
        self.fromLL_client = self.navigator.create_client(FromLL, '/fromLL')

        # Mission data
        self.waypoints: List[PoseStamped] = []

        # Feedback
        self.current_action = Mission.Feedback.NO_ACTION
        self.completion_status = Mission.Feedback.NOT_STARTED 

        # Action Server
        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.mission_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('MissionManager action server ready.')

    def gps_to_map_pose(self, lat, lon, yaw=0.0):           # KRJ TODO: Need to test how nav2 behaves if yaw on each point is 0
        """Use robot_localization to convert GPS to map pose"""
        
        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = 0.0
        
        # Wait for service
        self.fromLL_client.wait_for_service()
        
        # Call service
        future = self.fromLL_client.call_async(request)
        rclpy.spin_until_future_complete(self.navigator, future)
        
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

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received goal request: '
            f'search_object={goal_request.search_object}, '
            f'search_pattern={goal_request.search_pattern}, '
            f'num_waypoints={len(goal_request.nav_waypoints)}'
        )
        # Check if already busy
        if self.current_action != Mission.Feedback.NO_ACTION:
            self.get_logger().info('Rejecting new goal - already executing a mission')
            return GoalResponse.REJECT
        
        # Validate input
        if goal_request.search_pattern != Mission.Goal.NO_PATTERN:
            if goal_request.search_object not in [Mission.Goal.BOTTLE, Mission.Goal.OG_HAMMER, Mission.Goal.ORANGE_HAMMER]:
                self.get_logger().warn('Rejecting goal - invalid or missing search object')
                return GoalResponse.REJECT
        
        if goal_request.search_object != Mission.Goal.NO_OBJECT:
            if goal_request.search_pattern not in [Mission.Goal.SPIRAL]:
                self.get_logger().warn('Rejecting goal - invalid or missing search pattern')
                return GoalResponse.REJECT
        
        if len(goal_request.nav_waypoints) == 0:
            self.get_logger().warn('Rejecting goal - no waypoints provided')
            return GoalResponse.REJECT
        
        # Convert waypoints             KRJ TODO: Can this error?           
        self.waypoints = [self.gps_to_map_pose(gps.latitude, gps.longitude) for gps in goal_request.nav_waypoints]          # KRJ TODO: Is converting all at once bad if gps corrects?
        
        self.get_logger().info('Accepting goal request')
        self.current_action = Mission.Feedback.NAVIGATING
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    # ------------------------------------------------
    # Execute a mission
    # ------------------------------------------------
    async def mission_callback(self, goal_handle):
        
        class MissionCanceled(Exception):
            """Custom exception to handle mission cancellation"""
            pass
        
        def start_wpf(wps):
            """
            Function to start the waypoint following
            """
            self.completion_status = Mission.Feedback.IN_PROGRESS
            # self.navigator.waitUntilNav2Active(localizer='robot_localization')
            self.navigator.followWaypoints(wps)
            self.get_logger().info('Following waypoints')

        def wait_for_finish():
            """
            Wait for navigation to complete, checking for cancellation
            Raises MissionCanceled if cancel is requested
            """
            while not self.navigator.isTaskComplete():
                # Check for cancellation request
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Cancel requested - stopping navigation')
                    self.navigator.cancelTask()  # Stop the navigator
                    goal_handle.canceled()
                    raise MissionCanceled()
                
                # KRJ TODO: Do something with navigator feedback: self.navigator.getFeedback()

                # KRJ TODO: we should rethink feedback. This isn't very useful. Maybe send back completion percent and make a progress bar?
                
                # Send feedback
                feedback = Mission.Feedback()
                feedback.header = Header()
                feedback.current_action = self.current_action
                feedback.completion_status = self.completion_status
                goal_handle.publish_feedback(feedback)
                
                time.sleep(0.1)
            
            self.completion_status = Mission.Feedback.COMPLETED
            self.get_logger().info('Navigation step complete')
        
        try:                                        # KRJ TODO: set completion statuses and current actions
            # ==============================
            # Navigate to location
            # ==============================
            self.get_logger().info('Navigation to search location')
            start_wpf(self.waypoints)
            wait_for_finish()
            
            # ==============================
            # Execute search
            # ==============================
            goal_request = goal_handle.request
            if goal_request.search_object != Mission.Goal.NO_OBJECT and goal_request.search_pattern != Mission.Goal.NO_PATTERN:
                self.get_logger().info('Executing Search')
                self.current_action = Mission.Feedback.SEARCHING

                # KRJ TODO: call yolo node action server to begin looking for given object

                search_path = search_patterns.spiral(self.waypoints[-1], 10, 2)
                start_wpf(search_path)
                wait_for_finish()
            
            # ==============================
            # Complete action
            # ==============================
            self.get_logger().info('Mission Completed')
            goal_handle.succeed()
            result = Mission.Result()
            result.header = Header()
            result.ack = Mission.Result.SUCCESS
            return result
            
        except MissionCanceled:
            # Handle cancellation
            self.get_logger().info('Mission was canceled')
            result = Mission.Result()
            result.header = Header()
            result.ack = Mission.Result.CANCELED
            return result


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
