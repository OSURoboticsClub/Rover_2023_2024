#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_autonomy_interface.action import Mission
from nav_autonomy_interface.msg import GPSWaypoint

class MissionTestClient(Node):
    """
    Test client for Mission action server
    """
    def __init__(self):
        super().__init__('mission_test_client')
        
        self._action_client = ActionClient(self, Mission, 'mission')
        
        self.get_logger().info('Mission test client initialized')
    
    def send_goal(self, waypoints, search_object=Mission.Goal.ARUCO, 
                  search_pattern=Mission.Goal.NO_PATTERN):
        """Send a goal to the mission action server"""
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = Mission.Goal()
        goal_msg.nav_waypoints = waypoints
        goal_msg.search_object = search_object
        goal_msg.search_pattern = search_pattern
        
        self.get_logger().info(
            f'Sending goal with {len(waypoints)} waypoints, '
            f'search_object={search_object}, search_pattern={search_pattern}'
        )
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        
        action_names = {
            Mission.Feedback.NO_ACTION: 'NO_ACTION',
            Mission.Feedback.NAVIGATING: 'NAVIGATING',
            Mission.Feedback.SEARCHING: 'SEARCHING'
        }
        
        status_names = {
            Mission.Feedback.NOT_STARTED: 'NOT_STARTED',
            Mission.Feedback.IN_PROGRESS: 'IN_PROGRESS',
            Mission.Feedback.COMPLETED: 'COMPLETED'
        }
        
        self.get_logger().info(
            f'Feedback - Action: {action_names.get(feedback.current_action, "UNKNOWN")}, '
            f'Status: {status_names.get(feedback.completion_status, "UNKNOWN")}'
        )
    
    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        
        result_names = {
            Mission.Result.SUCCESS: 'SUCCESS',
            Mission.Result.CANCELED: 'CANCELED',
            Mission.Result.FAILED: 'FAILED'
        }
        
        self.get_logger().info(
            f'Result: {result_names.get(result.ack, "UNKNOWN")}'
        )
    
    def cancel_goal(self):
        """Cancel the current goal"""
        self.get_logger().info('Canceling goal...')
        if hasattr(self, '_send_goal_future'):
            goal_handle = self._send_goal_future.result()
            if goal_handle:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        """Handle cancel response"""
        cancel_response = future.result()
        self.get_logger().info(f'Cancel response: {cancel_response}')


def main(args=None):
    rclpy.init(args=args)
    
    client = MissionTestClient()
    
    # Example 1: Simple navigation waypoints (no search)
    waypoints = [
        GPSWaypoint(latitude=44.56728, longitude=-123.27430),  # New York
        GPSWaypoint(latitude=44.56736, longitude=-123.27426),  # Times Square
    ]
    
    # Example 2: Navigation with search
    # waypoints = [
    #     GPSWaypoint(latitude=40.7128, longitude=-74.0060),
    # ]
    # client.send_goal(
    #     waypoints, 
    #     search_object=Mission.Goal.BOTTLE,
    #     search_pattern=Mission.Goal.SPIRAL
    # )
    
    client.send_goal(waypoints)
    
    # To test cancellation, uncomment:
    # import threading
    # def cancel_after_delay():
    #     import time
    #     time.sleep(5.0)
    #     client.cancel_goal()
    # threading.Thread(target=cancel_after_delay, daemon=True).start()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
