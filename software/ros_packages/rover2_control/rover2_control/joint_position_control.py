#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions
import moveit_msgs.msg
import geometry_msgs.msg
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32MultiArray

class JointPositionController(Node):
    def __init__(self):
        super().__init__('joint_position_controller')
        
        # Create an action client for the MoveGroup action
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected!')
        self.joint_names = ['rover_arm_base_joint', 'rover_arm_shoulder_joint', 'rover_arm_elbow_pitch_joint', 
                  'rover_arm_elbow_roll_joint', 'rover_arm_wrist_pitch_joint', 'rover_arm_wrist_roll_joint']
        # Subscribe to current joint states for getting the current robot state
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)

        self.joint_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            '/set_joint_angles',
            self.move_to_joint_positions,
            10)
        self.latest_joint_state = None

    def joint_states_callback(self, msg):
        self.latest_joint_state = msg

    def move_to_joint_positions(self, msg):
        joint_positions = []
        for joint in msg.data:
            joint_positions.append(joint)
        # Wait until we have received joint states
        while self.latest_joint_state is None:
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # Create a motion planning request
        motion_request = MotionPlanRequest()
        motion_request.workspace_parameters.header.frame_id = "rover_arm_base_link"
        motion_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Set the planning group
        motion_request.group_name = "rover_arm"
        
        # Set start state to the current state
        motion_request.start_state = RobotState()
        motion_request.start_state.joint_state.header = self.latest_joint_state.header
        motion_request.start_state.joint_state.name = self.latest_joint_state.name
        motion_request.start_state.joint_state.position = self.latest_joint_state.position
        motion_request.start_state.joint_state.velocity = self.latest_joint_state.velocity
        
        # Set the goal constraints based on the desired joint positions
        motion_request.goal_constraints = [Constraints()]
        
        for i, joint_name in enumerate(self.joint_names):
            # Create a joint constraint for each joint
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_positions[i]
            joint_constraint.tolerance_above = 0.01  # radians
            joint_constraint.tolerance_below = 0.01  # radians
            joint_constraint.weight = 1.0
            
            # Add the joint constraint to the goal constraints
            motion_request.goal_constraints[0].joint_constraints.append(joint_constraint)
        
        # Set planning parameters
        motion_request.max_velocity_scaling_factor = 0.5
        motion_request.max_acceleration_scaling_factor = 0.5
        motion_request.allowed_planning_time = 5.0
        motion_request.num_planning_attempts = 10
        
        # Create planning options
        planning_options = PlanningOptions()
        planning_options.plan_only = False  # Set to True if you only want to plan without execution
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 5
        #planning_options.replan_delay = Duration(sec=2, nanosec=0)
        
        # Create the goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_request
        goal_msg.planning_options = planning_options
        
        # Send the goal
        self.get_logger().info(f'Sending goal to move joints {self.joint_names} to positions {joint_positions}')
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        
        # Add callbacks for goal response and result
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
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
        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    controller = JointPositionController()
    
    # Example: Move joints to specific positions
    # Replace these with your robot's actual joint names and desired positions
    # Keep the node running until it's interrupted
    rclpy.spin(controller)
    
    # Clean up
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
