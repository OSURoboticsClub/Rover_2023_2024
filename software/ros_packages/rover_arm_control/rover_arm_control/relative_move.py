"""
Relative Move Node
DAM Robotics
Authors: Jared Northrop
Year: 2526

This node implement the full moveit trajectory pipeline for relative end-effector movements. 

Notes
-----
Currently (11/17/25) relative orientation is not implemented and orientation is held fixed during movements.
In the current implementation this means that the end-effector will maintain its start or current 
orientation throughout the move. 
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, BoundingVolume, MoveItErrorCodes
import moveit_msgs.msg
from moveit_msgs.srv import GetPlanningScene

from geometry_msgs.msg import TwistStamped, Twist, PointStamped, Quaternion
from builtin_interfaces.msg import Duration
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rover_arm_control_interface.action import RelativeMove
from controller_manager_msgs.srv import SwitchController
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, Empty


from scipy.spatial.transform import Rotation as R

# from tf_transformations2 import quaternion_multiply, quaternion_matrix
#import tf_transformations

class GripperMoveNode(Node):
    def __init__(self):
        super().__init__('auton_typing_node')
        
        # get tf information
        # Setup (once in __init__)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb_group = MutuallyExclusiveCallbackGroup()


        # Create an action client for the MoveGroup action
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected!')

        self.controller_client = self.make_client(SwitchController, '/controller_manager/switch_controller')
        self.start_servo_client = self.make_client(Trigger, '/servo_node/start_servo')
        self.planning_scene_client = self.make_client(GetPlanningScene, '/get_planning_scene')



   
        # Subscribe to current joint states for getting the current robot state
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)

        # self.move_service = self.create_service(RelativeMove, "/relative_move", self.set_callback_flag)
        
        self.move_action = ActionServer(self, RelativeMove, 'relative_move', self.set_callback_flag)
        #set robot frames
        self.target_frame = "arm_gripper"
        self.reference_frame = "base_link"

        self.latest_joint_state = None

        self.current_pose = None
        #set up control booleans
        self.done_future = None
        self.is_done = False
        self.succeed = False
        self.in_service = False
        self.get_logger().info('GripperMoveNode is ready.')

    
    def make_client(self, srv_type, name):
        """ Create a client to a service.

        Parameters
        ----------
        srv_type : Service Type
            The service type defined in the .srv file.
        name : string
            The name of the service call.
        """
        client = self.create_client(srv_type, name, callback_group=self.cb_group)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{name}', retrying...")
        return client

    def joint_states_callback(self, msg):
        self.latest_joint_state = msg
    
    # def inverse_pose(self, offset_dict):
    #     new_dict = {}
    #     for key, val in offset_dict.items():
    #         pose = Pose()
    #         pose.position.x = val.position.x * -1
    #         pose.position.y = val.position.y * -1
    #         pose.position.z = val.position.z * -1
    #         pose.orientation.x = 0.0
    #         pose.orientation.y = 0.0
    #         pose.orientation.z = 0.0
    #         pose.orientation.w = 1.0 # Neutral orientation
    #         new_dict[key] = pose

    #     return new_dict  

    # def make_pose(self, pose_dict):
    #     for key in pose_dict:
    #         pose = Pose()
    #         #set array to x,y,z pose positions
    #         pose.position.x = pose_dict[key][0]
    #         pose.position.y = pose_dict[key][1]
    #         pose.position.z = pose_dict[key][2]
    #         #Set pose at 90 degree rotation along x
    #         pose.orientation.x = -0.7071068
    #         pose.orientation.y = 0.0
    #         pose.orientation.z = 0.0
    #         pose.orientation.w = 0.7071068  # Neutral orientation
    #         pose_dict[key] = pose
    #     return pose_dict

    def switch_controller(self, servo=False, sim=False):
        """Switch the ros2 control controllers.

        Parameters
        ----------
            servo : Bool
                Indicator for servo or trajectory controller
            sim : Bool
                Indicator for in sim mode (note: doesn't do anything). 

        """
        # Switches controller from forward position controller to joint_trajectory controller
        self.request = SwitchController.Request()
        self.get_logger().info("in switching controllers")
        if servo:
            if not sim:
                self.request.activate_controllers = ["rover_arm_controller"] 
                self.request.deactivate_controllers = ["rover_arm_controller_moveit"]
            else:
                self.request.activate_controllers = ["rover_arm_controller"] 
                self.request.deactivate_controllers = ["rover_arm_controller_moveit"]
            self.servo = True
        else:
            if not sim:
                self.request.activate_controllers = ["rover_arm_controller_moveit"]
                self.request.deactivate_controllers = ["rover_arm_controller"]
            else:
                self.request.activate_controllers = ["rover_arm_controller_moveit"]
                self.request.deactivate_controllers = ["rover_arm_controller"]
            self.servo = False
        self.request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        self.request.strictness = SwitchController.Request.BEST_EFFORT  # Use STRICT or BEST_EFFORT

        self.future = self.controller_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def start_servo(self):
            # Starts servo node
            self.request = Trigger.Request()
            self.future = self.start_servo_client.call_async(self.request)
            rclpy.spin_until_future_complete(self, self.future) 
            return self.future.result()
    
    def set_callback_flag(self, goal_handle):
        self.succeed = False

        #switch ros2 control controller to moveit
        self.switch_controller(servo=False)

        response = RelativeMove.Result()


        #setup moveit goal
        for i in range(2):
            goal_msg = self.setup_params(goal_handle.request.relative_pose.pose)


            #send goal and wait for return
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            self.get_logger().info("sent goal")
            rclpy.spin_until_future_complete(self, send_goal_future)
            sent_goal_handle = send_goal_future.result()

            #Check if goal was accepted and wait for result
            if not sent_goal_handle.accepted:
                self.get_logger().error('Goal was rejected!')
                self.succeed = False
                response.success = False
                response.message = "Goal was rejected"
                goal_handle.abort()
                return response

            self.get_logger().info('Goal accepted! Waiting for result...')

            # Wait for result
            result_future = sent_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result_handle = result_future.result()

            # Check result status
            status = result_handle.status
            if status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info('Motion execution succeeded!')
                self.succeed = True
                response.success = True
                response.message = "Motion Executed Successfully"
                break
            else:
                self.get_logger().error(f'Motion execution failed with status: {status}')
                self.succeed = False
                response.success = False
                response.message = "Unable to Execute Motion"
        if not self.succeed:
            goal_handle.abort()
        else: 
            goal_handle.succeed()
        self.get_logger().info(f"Goal complete. Success = {self.succeed}")

        #switch ros2 control controller to servo
        self.start_servo()
        self.switch_controller(servo=True)
        return response

    def get_EE_pose(self):
        # Get the current pose of the gripper
        try:
            trans = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                self.target_frame,  # source frame
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
    

    def compose_poses_ee(self, current: Pose, relative: Pose) -> Pose:
        """Computes the combining two poses
        """
        result = Pose()

        # --- orientation (quaternion composition) ---
        q_current = [current.orientation.x, current.orientation.y,
                    current.orientation.z, current.orientation.w]
        q_rel = [relative.orientation.x, relative.orientation.y,
                relative.orientation.z, relative.orientation.w]

        r1 = R.from_quat(q_current)
        r2 = R.from_quat(q_rel)

        # q_final = quaternion_multiply(q_current, q_rel)
        r3 = r1 * r2
        q_final = r3.as_quat()  # [x, y, z, w]
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = q_final

        # --- position ---
        # Rotate the relative position by the current orientation
        Rot = R.from_quat(q_current)[0:3, 0:3]  # 3x3 rotation matrix
        p_rel_rot = Rot.dot([relative.position.x, relative.position.y, relative.position.z])

        result.position.x = current.position.x + p_rel_rot[0]
        result.position.y = current.position.y + p_rel_rot[1]
        result.position.z = current.position.z + p_rel_rot[2]

        return result
    
    def get_current_robot_state(self) -> RobotState:
        req = GetPlanningScene.Request()
        # Empty request returns full scene including current robot state
        future = self.planning_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        scene = future.result().scene
        return scene.robot_state

    def create_motion_request(self):
        #create motion planning request
        motion_request = MotionPlanRequest()
        motion_request.workspace_parameters.header.frame_id = self.reference_frame
        motion_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()

        # Set the planning group
        motion_request.group_name = "rover_arm"

        #Define Planner
        motion_request.planner_id = "RRTConnectkConfigDefault"
        motion_request._pipeline_id = "ompl"

        # motion_request.planner_id = "PTP"
        # motion_request._pipeline_id = "pilz_industrial_motion_planner"

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
        motion_request.allowed_planning_time = 2.0
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
        


    def setup_params(self, relative_pose):
        """Build a moveit Pose request
        """
        # Create a motion planning request
        motion_request = self.create_motion_request()
        
        #get current pose
        self.get_EE_pose()

        #Set target pose
        target_pose = relative_pose
        #self.get_logger().info(f"Target Pose: {target_pose}")
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.pose = self.add_pose_position(target_pose, self.current_pose.pose)
        pose.pose.orientation = self.current_pose.pose.orientation
        #self.get_logger().info(f"Current: {self.current_pose.pose.position}")
        #self.get_logger().info(f"Goal: {pose.pose.position}")

        #create goal position contraint
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
        box.dimensions = [0.0005, 0.0005, 0.0005]  

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
        orientation_constraint.absolute_x_axis_tolerance = 0.0001
        orientation_constraint.absolute_y_axis_tolerance = 0.0001
        orientation_constraint.absolute_z_axis_tolerance = 0.0001
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

        # Create goal orientation constraint
        path_orientation_constraint = OrientationConstraint()
        path_orientation_constraint.header.frame_id = self.reference_frame
        path_orientation_constraint.link_name = self.target_frame
        path_orientation_constraint.orientation = self.current_pose.pose.orientation
        path_orientation_constraint.absolute_x_axis_tolerance = 0.01
        path_orientation_constraint.absolute_y_axis_tolerance = 0.01
        path_orientation_constraint.absolute_z_axis_tolerance = 0.01
        path_orientation_constraint.weight = 1.0


        #apply path constraint
        #motion_request.path_constraints.position_constraints.append(path_constraint)
        motion_request.path_constraints.orientation_constraints.append(path_orientation_constraint)

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
    # while rclpy.ok():
    #     if auton_type.in_service:
    #         pass
    #     else:
    #         rclpy.spin_once(auton_type)
    # executor = MultiThreadedExecutor(num_threads=4)
    # executor.add_node(auton_type)
    # executor.spin()
    rclpy.spin(auton_type)

    auton_type.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
