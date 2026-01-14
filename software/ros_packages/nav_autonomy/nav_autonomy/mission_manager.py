#!/usr/bin/env python3

from typing import List
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator

from std_msgs.msg import Header
from geographic_msgs.msg import GeoPose

from nav_autonomy.utils.gps_utils import latLonYaw2Geopose

# Custom action and message
from nav_autonomy_interface.action import Mission
from nav_autonomy_interface.msg import GPSWaypoint


def wps_to_geopose(waypoints: List[GPSWaypoint]) -> List[GeoPose]:
    """
    Convert array of give GPSWaypoint to geographic_msgs/msg/GeoPose 
    """
    gepose_wps = []
    for wp in waypoints:
        latitude, longitude, yaw = wp.latitude, wp.longitude, 0.0
        gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
    return gepose_wps


class MissionManager(Node):
    """
    MissionManager Action Server
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator("basic_navigator")

        # Mission data
        self.search_object: int = Mission.NONE
        self.search_pattern: int = Mission.NONE
        self.waypoints: List[GPSWaypoint] = []

        # Feedback
        self.current_action = Mission.NONE
        self.completion_status = Mission.NOT_STARTED 

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


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received goal request: '
            f'searchObject={goal_request.searchObject}, '
            f'searchPattern={goal_request.searchPattern}, '
            f'num_waypoints={len(goal_request.navWaypoints)}'
        )
        # Check if already busy
        if self.current_action != Mission.NONE:
            self.get_logger().info('Rejecting new goal - already executing a mission')
            return GoalResponse.REJECT
        
        # Validate input
        if len(goal_request.navWaypoints) == 0:
            self.get_logger().warn('Rejecting goal - no waypoints provided')
            return GoalResponse.REJECT
        # KRJ TODO: Verify lat long range?
        
        if goal_request.searchObject != Mission.NONE or goal_request.pattern != Mission.NONE:
            if goal_request.searchObject not in [Mission.CIRCLE, Mission.SQUARE]:
                self.get_logger().warn('Rejecting goal - no search object specified')
                return GoalResponse.REJECT
    
            if goal_request.searchPattern not in [Mission.BOTTLE, Mission.OG_HAMMER, Mission.ORANGE_HAMMER]:
                self.get_logger().warn('Rejecting goal - no search object specified')
                return GoalResponse.REJECT
        
        self.get_logger().info('Accepting goal request')
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
            self.navigator.waitUntilNav2Active(localizer='robot_localization')
            self.navigator.followGpsWaypoints(wps)
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

                # Send feedback
                feedback = Mission.Feedback()
                feedback.header = Header()
                feedback.currentAction = self.current_action
                feedback.completionStatus = self.completion_status
                goal_handle.publish_feedback(feedback)
                
                time.sleep(0.1)
            
            self.get_logger().info('Navigation step complete')
        
        try:                                        # KRJ TODO: set completion statuses and current actions
            # ==============================
            # Navigate to location
            # ==============================
            self.get_logger().info('Navigation to search location')
            self.current_action = Mission.NAVIGATING
            navigation_path = wps_to_geopose(self.waypoints)
            start_wpf(navigation_path)
            wait_for_finish()
            
            # ==============================
            # Execute search
            # ==============================
            if goal_handle.searchObject != Mission.NONE and goal_handle.pattern != Mission.NONE:
                self.get_logger().info('Executing Search')
                self.current_action = Mission.SEARCHING

                # KRJ TODO: call yolo node action server to begin looking for given object

                final_pose = navigation_path[-1]
                search_path = [final_pose]  # KRJ TODO: make search pattern, add final pose to each item in search path
                start_wpf(search_path)

                wait_for_finish()
            
            # ==============================
            # Complete action
            # ==============================
            self.get_logger().info('Mission Completed')
            goal_handle.succeed()
            result = Mission.Result()
            result.header = Header()
            result.ackComplete = 1  # ACK success
            return result
            
        except MissionCanceled:
            # Handle cancellation
            self.get_logger().info('Mission was canceled')
            result = Mission.Result()
            result.header = Header()
            result.ackComplete = 0  # Indicate cancellation
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
