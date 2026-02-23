import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rover_arm_control_interface.action import RelativeMove, AutonTyping
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse



class TestAutonTyping(Node):

    def __init__(self):
        super().__init__('test_auton_typing')
        self.auton_typing_client = ActionClient(self, AutonTyping, 'auton_typing')
        self.get_logger().info('Waiting for AutonTyping action server...')
        self.auton_typing_client.wait_for_server()
        self.get_logger().info('AutonTyping action server connected!')

        self.message = AutonTyping.Goal()
        self.message.phrase = "d,a,m,r,o,b,o,t,i,c,s"
        self.send_future_goal = self.auton_typing_client.send_goal_async(self.message)
        self.send_future_goal.add_done_callback(self.goal_response_callback)

        self.is_done = False
        self.move_success = False



    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.is_done = True
            return
            
        self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status        
        if status == 4:  # Succeeded
            self.get_logger().info('Motion execution succeeded!')
            self.move_success = True

        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')
            self.move_success = False
        self.is_done = True



def main(args=None):
    rclpy.init(args=args)

    test_auton_typing = TestAutonTyping()

    rclpy.spin(test_auton_typing)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_auton_typing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()