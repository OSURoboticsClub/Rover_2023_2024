"""
Auton Typing Node
DAM Robotics
Authors: Jared Northrop
Year: 2526

This node implements autonomous typing for the University Rover Challenge's equipment servicing mission. A 
string is sent through an action call where each key is comma seperated. 

Notes
-----
- See dictionary for key names. 
- Key poses are hardcoded offsets from the Q key. 
- Current implementation always moves back to the Q key before moving to the next key in the phrase. 

"""
import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import String
from rover_arm_control_interface.action import RelativeMove, AutonTyping
from rclpy.callback_groups import ReentrantCallbackGroup
import csv
from io import StringIO
import time

from geometry_msgs.msg import PoseStamped



class AutonomousTyping(Node):

    def __init__(self):
        super().__init__('auton_typing_node')

        self.relative_move_client = ActionClient(self, RelativeMove, 'relative_move')
        self.get_logger().info('Waiting for RelativeMove action server...')
        self.relative_move_client.wait_for_server()
        self.get_logger().info('RelativeMove action server connected!')


        self.auton_typing_action = ActionServer(
            node=self, 
            action_type=AutonTyping, 
            action_name='auton_typing', 
            goal_callback=self.accept_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.goal_callback,
        )

        self.key_dict = {
            # Row 1 (Escape and Function keys)
            'Esc': [-0.02775, 0.043],
            'F1': [0.01175, 0.043], 'F2': [0.0185+0.01175, 0.043], 'F3': [2*0.0185+0.01175, 0.043], 'F4': [3*0.0185+0.01175, 0.043],
            'F5': [0.09425, 0.043], 'F6': [0.0185+0.09425, 0.043], 'F7': [2*0.0185+0.09425, 0.043], 'F8': [3*0.0185+0.09425, 0.043],
            'F9': [0.17575, 0.043], 'F10': [0.0185+0.17575, 0.043], 'F11': [2*0.0185+0.17575, 0.043], 'F12': [3*0.0185+0.17575, 0.043],
            'PrtSc': [0.25575, 0.043], 'ScrollLock': [0.0185+0.25575, 0.043], 'Pause': [2*0.0185+0.25575, 0.043],

            # Row 2 (Tilde to Backspace)
            '`': [-0.02775, 0.0185], '1': [0.0185-0.02775, 0.0185], '2': [2*0.0185-0.02775, 0.0185], '3': [3*0.0185-0.02775, 0.0185],
            '4': [4*0.0185-0.02775, 0.0185], '5': [5*0.0185-0.02775, 0.0185], '6': [6*0.0185-0.02775, 0.0185], '7': [7*0.0185-0.02775, 0.0185],
            '8': [8*0.0185-0.02775, 0.0185], '9': [9*0.0185-0.02775, 0.0185], '0': [10*0.0185-0.02775, 0.0185], '-': [11*0.0185-0.02775, 0.0185],
            '=': [12*0.0185-0.02775, 0.0185], 'Backspace': [13*0.0185-0.02775, 0.0185],

            # Row 3 (Tab to Backslash)
            'Tab': [-0.0185, 0.0], 'Q': [0.0, 0.0], 'W': [0.0185, 0.0], 'E': [2*0.0185, 0.0],
            'R': [3*0.0185, 0.0], 'T': [4*0.0185, 0.0], 'Y': [5*0.0185, 0.0], 'U': [6*0.0185, 0.0],
            'I': [7*0.0185, 0.0], 'O': [8*0.0185, 0.0], 'P': [9*0.0185, 0.0], '[': [10*0.0185, 0.0],
            ']': [11*0.0185, 0.0], '\\': [9*0.0185, 0.0],

            # Row 4 (Caps Lock to Enter)
            'CapsLock': [-0.0185+.00525, -0.0185], 'A': [.00525, -0.0185], 'S': [0.0185+.00525, -0.0185], 'D': [2*0.0185+.00525, -0.0185],
            'F': [3*0.0185+.00525, -0.0185], 'G': [4*0.0185+.00525, -0.0185], 'H': [5*0.0185+.00525, -0.0185], 'J': [6*0.0185+.00525, -0.0185],
            'K': [7*0.0185+.00525, -0.0185], 'L': [8*0.0185+.00525, -0.0185], ';': [9*0.0185+0.01475, -0.037], '\'': [10*0.0185+0.01475, -0.037],
            'Enter': [11*0.0185+0.01475, -0.037],

            # Row 5 (Left Shift to Right Shift)
            'LShift': [-0.0185+0.01475, -0.037], 'Z': [0.01475, -0.037], 'X': [0.0185+0.01475, -0.037], 'C': [2*0.0185+0.01475, -0.037],
            'V': [3*0.0185+0.01475, -0.037], 'B': [4*0.0185+0.01475, -0.037], 'N': [5*0.0185+0.01475, -0.037], 'M': [6*0.0185+0.01475, -0.037],
            ',': [7*0.0185+0.01475, -0.037], '.': [8*0.0185+0.01475, -0.037], '/': [9*0.0185+0.01475, -0.037], 'RShift': [10*0.0185+0.01475, -0.037],

            # Row 6 (Left Ctrl to Right Ctrl)
            'LCtrl': [None, None], 'LWin': [None, None], 'LAlt': [None, None], 'SPACE': [None, None],
            'RAlt': [None, None], 'RWin': [None, None], 'Menu': [None, None], 'RCtrl': [None, None],

            # Navigation Cluster
            'Insert': [None, None], 'Home': [None, None], 'PageUp': [None, None],
            'Delete': [None, None], 'End': [None, None], 'PageDown': [None, None],

            # Arrow Keys
            'Up': [None, None], 'Left': [None, None], 'Down': [None, None], 'Right': [None, None],
        }

        self.phrase = None

        #state variables
        self.is_done = False
        self.move_success = False

    def accept_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.phrase = None
        return CancelResponse.ACCEPT
    
    def goal_callback(self, goal_handle):
        self.get_logger().info(f"Recieved Phrase: {goal_handle.request.phrase}")
        self.phrase = self.split_csv(goal_handle.request.phrase)
        self.get_logger().info(f"Decoded Phrase: {self.phrase}")
        request_msg = RelativeMove.Goal()
        succeed = False
        for i, letter in enumerate(self.phrase):
            self.is_done = False
            self.move_success = False
            self.get_logger().info(f"Sending key: {letter}")
            request_msg.relative_pose = self.get_letter_pose(letter.upper())
            send_goal_future = self.relative_move_client.send_goal_async(request_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            #Move to key offset
            while not self.is_done:
                rclpy.spin_once(self)
                feedback = AutonTyping.Feedback()
                feedback.feedback = f"Moving to {letter}"
                goal_handle.publish_feedback(feedback)
                time.sleep(0.02)
            #Push key
            if self.move_success:
                self.is_done = False
                self.move_success = False
                request_msg.relative_pose = self.make_posestamped([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0])
                send_goal_future = self.relative_move_client.send_goal_async(request_msg)
                send_goal_future.add_done_callback(self.goal_response_callback)
                while not self.is_done:
                    rclpy.spin_once(self)
                    feedback = AutonTyping.Feedback()
                    feedback.feedback = f"Pushing the key"
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.02)

            #Retract
            if self.move_success:
                self.is_done = False
                self.move_success = False
                request_msg.relative_pose = self.make_posestamped([0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 1.0])
                send_goal_future = self.relative_move_client.send_goal_async(request_msg)
                send_goal_future.add_done_callback(self.goal_response_callback)
                while not self.is_done:
                    rclpy.spin_once(self)
                    feedback = AutonTyping.Feedback()
                    feedback.feedback = f"Pushing the key"
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.02)

            #move back to home
            if self.move_success:
                self.is_done = False
                self.move_success = False
                request_msg.relative_pose = self.get_letter_pose(letter.upper(), invert=True)
                send_goal_future = self.relative_move_client.send_goal_async(request_msg)
                send_goal_future.add_done_callback(self.goal_response_callback)
                while not self.is_done:
                    rclpy.spin_once(self)
                    feedback = AutonTyping.Feedback()
                    feedback.feedback = f"Pushing the key"
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.02)
        response = AutonTyping.Result()
        response.message = "Testing Auton"
        response.success = True

        succeed = True
        if not succeed:
            goal_handle.abort()
            self.get_logger().info("Failed to complete movements")
        else:
            goal_handle.succeed()
            self.get_logger().info("Completed Movements")

        return response
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.is_done = True
            return
            
        # self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status        
        if status == 4:  # Succeeded
            # self.get_logger().info('Motion execution succeeded!')
            self.move_success = True
            self.is_done = True

        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')
            self.move_success = False
            self.is_done = True
    
    def get_letter_pose(self, letter, invert=False):
        sign = 1
        if invert:
            sign = -1
        pose = PoseStamped()
        pose.header.frame_id = "rover_arm_tool0"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = sign * self.key_dict[letter][0]
        pose.pose.position.y =  -sign * self.key_dict[letter][1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose
    
    def make_posestamped(self, pose_arr):
        """Build a pose stamped message from an array.

        Parameters
        ----------
        pose_arr : list
            A list of pose values [x, y, z, qx, qy, qz, qw].

        Returns
        -------
        pose : PoseStamped
            The constructed PoseStamped message.
        
        Notes
        -----
        A header frame id should be added as a parameter.
        """
        pose = PoseStamped()
        pose.header.frame_id = "rover_arm_tool0"
        #set array to x,y,z pose positions
        pose.pose.position.x = pose_arr[0]
        pose.pose.position.y = pose_arr[1]
        pose.pose.position.z = pose_arr[2]
        #Set pose at 90 degree rotation along x
        pose.pose.orientation.x = pose_arr[3]
        pose.pose.orientation.y = pose_arr[4]
        pose.pose.orientation.z = pose_arr[5]
        pose.pose.orientation.w = pose_arr[6]  # Neutral orientation
        return pose
        

    def split_csv(self, phrase):
        return next(csv.reader(StringIO(phrase), delimiter=',', escapechar="\\"))



def main(args=None):
    rclpy.init(args=args)

    auton_typing_node = AutonomousTyping()

    rclpy.spin(auton_typing_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auton_typing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()