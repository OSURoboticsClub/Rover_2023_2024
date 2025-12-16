import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, BoundingVolume, MoveItErrorCodes
import moveit_msgs.msg
import geometry_msgs.msg
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
#import tf_transformations

class GripperMoveNode(Node):
    def __init__(self):
        super().__init__('auton_typing_node')
        
        # get tf information
        # Setup (once in __init__)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create an action client for the MoveGroup action
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected!')
        #set robot frames
        self.target_frame = "arm_gripper"
        self.reference_frame = "base_link"
        # Subscribe to current joint states for getting the current robot state
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.keyboard_offsets = {
            "q" : [0.05,0.0,0.0],
            "w" : [-0.2, 0.0, 0.0],
        }
        self.keyboard_offsets = self.make_pose(self.keyboard_offsets)
        #self.get_logger().info(self.keyboard_offsets)

        self.keyboard_offsets_inverse = self.inverse_pose(self.keyboard_offsets)

        self.push_button = {
	    "push" :  [0.0, 0.0, -0.05],
	    "back" :  [0.0, 0.0, 0.05]
      
	    } #x, y, z for pressing a button
        self.push_button = self.make_pose(self.push_button)

        self.latest_joint_state = None

        # Subscribe to the string command topic
        self.subscription = self.create_subscription(
            String,
            '/auton_typing',
            self.set_callback_flag,
            10
        )
        self.current_pose = None
        self.command = ""
        #set up control booleans
        self.is_done = False
        self.succeed = False
        self.is_callback = False

        self.get_logger().info('GripperMoveNode is ready.')

    def joint_states_callback(self, msg):
        self.latest_joint_state = msg
    
    def inverse_pose(self, offset_dict):
        new_dict = {}
        for key, val in offset_dict.items():
            pose = Pose()
            pose.position.x = val.position.x * -1
            pose.position.y = val.position.y * -1
            pose.position.z = val.position.z * -1
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0 # Neutral orientation
            new_dict[key] = pose

        return new_dict  

    def make_pose(self, pose_dict):
        for key in pose_dict:
            pose = Pose()
            #set array to x,y,z pose positions
            pose.position.x = pose_dict[key][0]
            pose.position.y = pose_dict[key][1]
            pose.position.z = pose_dict[key][2]
            #Set pose at 90 degree rotation along x
            pose.orientation.x = -0.7071068
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.7071068  # Neutral orientation
            pose_dict[key] = pose
        return pose_dict

    
    def set_callback_flag(self, msg):
        self.is_callback = True
        self.command = msg.data.lower().strip()
        return
    
    def text_callback(self):
        #command = msg.data.lower().strip()
        self.get_logger().info(f"What was recieved {self.command}")
        i = len(self.command)
        for key_input in self.command:
            self.get_logger().info(f"what is key {key_input}")
            if key_input in self.keyboard_offsets:
                goal_msg = self.setup_params(self.keyboard_offsets, key_input)

                #Move to key
                j = 0
                self.get_logger().info("Moving to key")
                while not self.succeed and j < 5:
                    self.is_done = False
                    self.send_goal(goal_msg, "push")
                    while not self.is_done:
                        continue
                    j += 1
                self.succeed = False
                self.is_done = False
                
                #push key
                self.get_logger().info("Push")
                goal_msg = self.setup_params(self.push_button, "push")
                j = 0
                while not self.succeed and j < 5:
                    self.is_done = False
                    self.send_goal(goal_msg, "push")
                    while not self.is_done:
                        continue
                    j += 1
                self.succeed = False
                self.is_done = False
                
                #return back
                self.get_logger().info("Back")
                goal_msg = self.setup_params(self.push_button, "back")
                j = 0
                while not self.succeed and j < 5:
                    self.is_done = False
                    self.send_goal(goal_msg, "push")
                    while not self.is_done:
                        continue
                    j += 1
                self.is_done = False
                self.succeed = False

                #return to reference key
                self.get_logger().info("Return to Home")

                goal_msg = self.setup_params(self.keyboard_offsets_inverse, key_input)
                j = 0
                while not self.succeed and j < 5:
                    self.is_done = False
                    self.send_goal(goal_msg, "push")
                    while not self.is_done:
                        continue
                    j += 1
                self.succeed = False
                self.is_done = False
                self.get_logger().info("Done with commands")
            else:
                self.get_logger().warn(f"Unknown command: '{key_input}'")

    def push_key(self):
        return

    def send_goal(self, goal_msg, key_input):
        #Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.moving_callback)
        while not self.is_done:
            rclpy.spin_once(self)

    def moving_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.is_done = True
            return
        self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        while not self.is_done:
            rclpy.spin_once(self)
        self.get_logger().info(f"{self.succeed}")
    
    def get_result_callback(self, future):
        result_handle = future.result()
        status = future.result().status
        self.get_logger().info(f"{status}")
        if status == 4:  # Succeeded
            self.get_logger().info('Motion execution succeeded!')
            self.succeed = True
        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')
            #self.succeed = True #don't care about trying multiple times
        self.is_done = True
        

    def get_pose(self):
        # Get the current pose of the gripper
        try:
            trans = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                "arm_gripper",  # source frame
                rclpy.time.Time())  # latest available

            self.current_pose = PoseStamped()
            self.current_pose.header = trans.header
            self.current_pose.pose.position.x = trans.transform.translation.x
            self.current_pose.pose.position.y = trans.transform.translation.y
            self.current_pose.pose.position.z = trans.transform.translation.z
            self.current_pose.pose.orientation = trans.transform.rotation

            # Now you have the current gripper pose
            #self.get_logger().info(f"Gripper pose: {self.current_pose.pose}")
        except Exception as e:
            self.get_logger().error(f"Failed to lookup gripper pose: {str(e)}")

    def add_pose_position(self, pose1, pose2):
        pose = Pose()
        pose.position.x = pose1.position.x + pose2.position.x
        pose.position.y = pose1.position.y + pose2.position.y
        pose.position.z = pose1.position.z + pose2.position.z
        pose.orientation.x = pose1.orientation.x
        pose.orientation.y = pose1.orientation.y
        pose.orientation.z = pose1.orientation.z
        pose.orientation.w = pose1.orientation.w
        return pose

    def create_motion_request(self):
        #create motion planning request
        motion_request = MotionPlanRequest()
        motion_request.workspace_parameters.header.frame_id = self.reference_frame
        motion_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()

        # Set the planning group
        motion_request.group_name = "rover_arm"

        #Define Planner
        motion_request.planner_id = "PTP"
        motion_request._pipeline_id = "pilz_industrial_motion_planner"

        # Set start state to the current state
        motion_request.start_state = RobotState()
        motion_request.start_state.joint_state.header = self.latest_joint_state.header
        motion_request.start_state.joint_state.name = self.latest_joint_state.name
        motion_request.start_state.joint_state.position = self.latest_joint_state.position
        motion_request.start_state.joint_state.velocity = self.latest_joint_state.velocity

        #planning params
        #set planning params
        motion_request.max_velocity_scaling_factor = 0.5
        motion_request.max_acceleration_scaling_factor = 0.5
        motion_request.allowed_planning_time = 5.0
        motion_request.num_planning_attempts = 10

        #Initialize Constraints
        motion_request.goal_constraints = [Constraints()]
        # motion_request.goal_constraints = [Constraints(
        #     position_constraints=[],
        #     orientation_constraints=[],
        #     joint_constraints=[]
        # )]

        motion_request.path_constraints = Constraints()

        return motion_request
        


    def setup_params(self, key_dict, key_input):
        # Create a motion planning request
        motion_request = self.create_motion_request()
        
        #get current pose
        self.get_pose()

        #Set target pose
        target_pose = key_dict[key_input]
        #self.get_logger().info(f"Target Pose: {target_pose}")
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.pose = self.add_pose_position(target_pose, self.current_pose.pose)
        pose.pose.orientation = self.current_pose.pose.orientation
        #self.get_logger().info(f"Current: {self.current_pose.pose.position}")
        #self.get_logger().info(f"Goal: {pose.pose.position}")

        #create goal osition contraint
        pose_constraint = PositionConstraint()
        pose_constraint.header.frame_id = pose.header.frame_id
        pose_constraint.link_name = self.target_frame
        pose_constraint.target_point_offset.x = 0.0
        pose_constraint.target_point_offset.y = 0.0
        pose_constraint.target_point_offset.z = 0.0

        #create bounding box: target point must lie with in (thus accuracy)
        bounding_volume = BoundingVolume()

        # Create box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.005, 0.005, 0.005]  

        # Assign the primitive to bounding volume
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses. append(pose.pose)
        pose_constraint.constraint_region = bounding_volume
        
        #set importance of the constraint
        pose_constraint.weight = 1.0
        
        # Create goal orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.link_name = self.target_frame
        orientation_constraint.orientation = self.current_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        
        #Add goal constraints
        motion_request.goal_constraints[0].position_constraints.append(pose_constraint)
        motion_request.goal_constraints[0].orientation_constraints.append(orientation_constraint)

        #Create Path constraint
        path_constraint = PositionConstraint()
        path_constraint.header.frame_id = "base_link"  # or your world frame
        path_constraint.link_name = self.target_frame
        path_constraint.target_point_offset.x = 0.0
        path_constraint.target_point_offset.y = 0.0
        path_constraint.target_point_offset.z = 0.0
        path_constraint.weight = 1.0

        # Define a very thin box in Z to constrain to XY plane at current Z
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 1.0, 2.0]  # large xz, thin y

        box_pose = Pose()
        box_pose.position.x = self.current_pose.pose.position.x
        box_pose.position.y = self.current_pose.pose.position.y - box.dimensions[1] / 2 + 0.001
        box_pose.position.z = self.current_pose.pose.position.z  # Z-plane to stay in
        box_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses.append(box_pose)

        path_constraint.constraint_region = bounding_volume

        #apply path constraint
        #motion_request.path_constraints.position_constraints.append(path_constraint)
        #motion_request.path_constraints.orientation_constraints.append(orientation_constraint)

        #set planning
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
        return goal_msg


def main(args=None):
    rclpy.init(args=args)
    auton_type = GripperMoveNode()
    while rclpy.ok():
        if auton_type.is_callback:
            auton_type.is_callback = False
            auton_type.text_callback()
        rclpy.spin_once(auton_type)


    auton_type.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
