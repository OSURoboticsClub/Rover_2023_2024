#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header

# Custom action and message
from nav_autonomy_interface.action import Mission
from nav_autonomy_interface.msg import GPSWaypoint


class MissionManager(Node):
    """
    MissionManager Action Server

    Responsibilities:
    - Receive mission definition (all waypoints at once)
    - Store mission data internally
    - Later: send waypoints incrementally to another node
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.callback_group = ReentrantCallbackGroup()

        # Stored mission data
        self.search_object: int = Mission.NONE
        self.search_pattern: int = Mission.NONE
        self.waypoints: List[GPSWaypoint] = []

        # Action Server
        self._action_server = ActionServer(
            self,
            Mission,
            'mission',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info('MissionManager action server ready.')

    # ------------------------------------------------
    # Action execution
    # ------------------------------------------------
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received new mission request')

        # ==============================
        # Store mission data
        # ==============================
        self.search_object = goal_handle.request.searchObject
        self.search_pattern = goal_handle.request.searchPattern
        self.waypoints = list(goal_handle.request.navWaypoints)

        self.get_logger().info(
            f'Mission stored: '
            f'searchObject={self.search_object}, '
            f'searchPattern={self.search_pattern}, '
            f'num_waypoints={len(self.waypoints)}'
        )

        # ==============================
        # Placeholder: process mission
        # ==============================
        feedback = Mission.Feedback()
        feedback.header = Header()
        feedback.currentAction = 0
        feedback.completionStatus = 0

        # Example feedback update (you will replace this)
        goal_handle.publish_feedback(feedback)

        # NAVIGATE TO SEARCH LOCATION
        # TODO: GIVE all waypoints to /FollowWaypoints or /NavigateThroughPoses or /FollowGPSWaypoints
            # Convert to respective types
            # Record the FollowWaypoints action feedback (current waypoint) and relay feedback for mission back to client

        # SEARCH
        # TODO: Give alll waypoints for the search path to /FollowWaypoints
            # Create list of points based on final pose in list and selected search pattern

        # ==============================
        # Complete action
        # ==============================
        goal_handle.succeed()

        result = Mission.Result()
        result.header = Header()
        result.ackComplete = 1  # ACK success

        self.get_logger().info('Mission accepted and stored successfully')

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
