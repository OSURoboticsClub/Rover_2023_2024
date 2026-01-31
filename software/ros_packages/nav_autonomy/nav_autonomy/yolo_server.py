#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header

from nav_autonomy_interface.action import YoloFind
from nav_autonomy_interface.msg import YoloInfo


class YoloServer(Node):
    """
    Yolo wrapper - Action server for YoloFind
    """

    def __init__(self):
        super().__init__('yolo_server')

        self.callback_group = ReentrantCallbackGroup()

        self.busy = False       # Is a search already being executed?

        # Action Server
        self._action_server = ActionServer(
            self,
            YoloFind,
            'YoloFind',
            execute_callback=self.action_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('YoloServer action server ready.')

        self.info_publisher = self.create_publisher(
            YoloInfo,
            'yolo_info',
            10
        )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received goal request: '
            f'search_object={goal_request.search_object}, '
        )
        # Check if already busy
        if self.busy:
            self.get_logger().info('Rejecting new goal - already executing a search')
            return GoalResponse.REJECT
        
        # Validate input
        if goal_request.search_object not in [YoloFind.Goal.BOTTLE, YoloFind.Goal.OG_HAMMER, YoloFind.Goal.ORANGE_HAMMER]:
            self.get_logger().warn('Rejecting goal - invalid or missing search object')
            return GoalResponse.REJECT

        self.get_logger().info('Accepting goal request')
        self.busy = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def action_callback(self, goal_handle):
        
        class ActionCanceled(Exception):
            """Custom exception to handle action cancellation"""
            pass
        
        try:
            # ==============================
            # Look for object
            # ==============================
            # TODO: Start Yolo here


            # TODO: Intermittently send feedback
            
            # ==============================
            # Complete action
            # ==============================
            self.get_logger().info('Yolo search Completed')
            goal_handle.succeed()
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.SUCCESS
            self.busy = False
            return result
            
        except ActionCanceled:
            # Handle cancellation
            self.get_logger().info('Yolo search canceled')
            result = YoloFind.Result()
            result.header = Header()
            result.ack = YoloFind.Result.CANCELED
            self.busy = False
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


if __name__ == '__main__':
    main()
