"""
Pick and Place Node
DAM Robotics
Authors: Jared Northrop
Year: 2526

This node implements a pick and place routine with object selection via human input through pointing an Aruco
marker. This has been built for ROB514 Introduction to Robotics course project. 
"""
import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, PointStamped
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState, Joy, PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger, Empty
from controller_manager_msgs.srv import SwitchController

from tf2_ros import Buffer, TransformListener

#moveit stuff
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions

#custom action call
from rover_arm_control_interface.action import RelativeMove, GripperControl
from pc_processing.srv import ResetObjects



#python stuff
import time
from collections import deque
import numpy as np

class SquareMakingController(Node):

    def __init__(self):
        super().__init__('square_maker')
        self.get_clock().now()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb_group = MutuallyExclusiveCallbackGroup()


        #clients
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected!')

        self.relative_move_client = ActionClient(self, RelativeMove, 'relative_move')
        self.get_logger().info('Waiting for RelativeMove action server...')
        self.relative_move_client.wait_for_server()
        self.get_logger().info('RelativeMove action server connected!')

        self.absolute_move_client = ActionClient(self, RelativeMove, 'absolute_move')
        self.get_logger().info('Waiting for AbsoluteMove action server...')
        self.absolute_move_client.wait_for_server()
        self.get_logger().info('AbsoluteMove action server connected!')

        self.gripper_control_client = ActionClient(self, GripperControl, 'gripper_control')
        self.get_logger().info('Waiting for GripperControl action server...')
        self.absolute_move_client.wait_for_server()
        self.get_logger().info('GripperControl action server connected!')


        self.controller_client = self.make_client(SwitchController, '/controller_manager/switch_controller')
        self.configure_servo_cli = self.make_client(SetParameters, '/servo_node/set_parameters')
        self.start_servo_client = self.make_client(Trigger, '/servo_node/start_servo')
        self.start_object_cli = self.make_client(ResetObjects, '/reset_pc_processing')
        self.set_plane_cli = self.make_client(Trigger, '/set_ground_plane')

        #Publishers
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)
        self.joy_pub_ = self.create_publisher(Joy, '/joy2', 1)

        #subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        
        self.objects_sub = self.create_subscription(
            PointCloud2,
            "/obj_centroids",
            self.objects_callback,
            10
        )

        self.pointing_sub = self.create_subscription(
            PointStamped,
            "/aruco/ground_intersection",
            self.pointing_callback,
            10
        )

        #timers
        self.timer = self.create_timer(0.03, self.timer_callback)

        #state params
        self.state = "start"
        self.latest_joint_state = None
        self.move_success = False
        self.sent_goal = False
        self.servo = True

        #define scan params
        self.sequence = {"down":"right", "right":"up", "up":"left", "left":"down"}
        self.dir_def = {"down":[0.0,-0.1], "right":[-0.1,0.0], "up":[0.0,0.1], "left":[0.1, 0.0]}
        self.dir_time = {"down":4000, "right":8000, "up":4000, "left":8000}
        self.curr_dir = "down"
        self.loops = 0
        self.cycles = 0
        self.sq_keys = ["down", "right", "up", "left"]
        self.scan_iter = 0

        #Square sides [m]
        self.square_dict = {
            "down" : self.make_posestamped([-0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "up" : self.make_posestamped([0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "right" : self.make_posestamped([0.0, -0.75, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "left" : self.make_posestamped([0.0 ,0.75, 0.0, 0.0, 0.0, 0.0, 0.0])
        }

        #Arm params
        self.frame_id = "rover_arm_gripper" # usually "arm_gripper"
        self.joint_names = ['rover_arm_base_joint', 'rover_arm_shoulder_joint', 'rover_arm_elbow_pitch_joint', 
                  'rover_arm_elbow_roll_joint', 'rover_arm_wrist_pitch_joint', 'rover_arm_wrist_roll_joint']
        self.gripper_offset = 0.335
        
        #Visual segmentation params
        self.object_pose = None
        self.object_list = []

        #Ground intersection point params
        self.deviation_threshold = 0.1  # meters
        self.points = deque(maxlen=10)  # Store the last 10 points
        self.points_mean = np.zeros(3)
        self.points_std = 100 * np.ones(3)
        self.within_radius = 0.2  # meters
        self.pick_height = None

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
        if servo:
            if not sim:
                self.request.activate_controllers = ["rover_arm_velocity_controller"] 
                self.request.deactivate_controllers = ["rover_arm_controller_moveit"]
            else:
                self.request.activate_controllers = ["rover_arm_velocity_controller"] 
                self.request.deactivate_controllers = ["rover_arm_controller_moveit"]
            self.servo = True
        else:
            if not sim:
                self.request.activate_controllers = ["rover_arm_controller_moveit"]
                self.request.deactivate_controllers = ["rover_arm_velocity_controller"]
            else:
                self.request.activate_controllers = ["rover_arm_controller_moveit"]
                self.request.deactivate_controllers = ["rover_arm_velocity_controller"]
            self.servo = False
        self.request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        self.request.strictness = SwitchController.Request.BEST_EFFORT  # Use STRICT or BEST_EFFORT

        self.future = self.controller_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def configure_servo(self, frame):
        """Change the moveit servo planning frame.

        Parameters
        ----------
        frame : string
            The name of the planning frame.
        """
        # Changes planning frame of the servo node
        #arguments: "base_link" for base frame, "tool0" for tool frame
        req = SetParameters.Request()

        new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=frame)
        req.parameters = [Parameter(name='moveit_servo.robot_link_command_frame', value=new_param_value)]
        self.future = self.configure_servo_cli.call_async(req)

    def joint_states_callback(self, msg):
        """Callback function to stores joint states.
        """
        self.latest_joint_state = msg

    def start_servo(self):
        """Start moveit servo.

        Returns
        -------
        future.result : Service Response
            The result from the service call.
        """
        # Starts servo node
        self.request = Trigger.Request()
        self.future = self.start_servo_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()
    
    def reset_count_object(self, enable=True, reset=True):
        """Service call to reset the object detection node.

        Parameters
        ----------
        enable : Bool
            Indicator to enable or disable object detection.
        reset : Bool
            Indicator to reset the detected objects.
        
        Returns
        -------
        future.result : Service Response
            The response from the service call.
        """
        self.request = ResetObjects.Request()
        self.request.enable = enable
        self.request.reset = True
        self.future = self.start_object_cli.call_async(self.request)

        return self.future.result()
    
    def objects_callback(self, msg):
        """Callback for object centroids.

        This function receives a PointCloud2 message containing the centroids of detected objects and stores
        them in self.object_list as PoseStamped messages.

        Parameters
        ----------
        msg : PointCloud2
            The point cloud message containing object centroids.
        """
        object_data = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.get_logger().info(f"Points {object_data}.")
        if object_data == []:
            return
        self.object_list = []
        for point in object_data:
            # self.get_logger().info(f"Point {type(point.x)}.")
            self.object_list.append(self.make_posestamped([float(point.x), float(point.y), float(point.z) + self.gripper_offset, 0.7071068, 0.7071068, 0.0, 0.0]))
        
    def pointing_callback(self, msg):
        """Callback for recieving the current ground intersection point from aruco detector.
        
        Parameters
        ----------
        msg : PointStamped
            The point message containing the ground intersection point.
        """
        self.points.append(np.array([msg.point.x, msg.point.y, msg.point.z]))
        if len(self.points) > 0:
            self.points_mean = np.mean(self.points, axis=0)
            self.points_std = np.std(self.points, axis=0)
            # self.get_logger().info(f"Pointing mean: {self.points_mean}, std: {self.points_std}")

    def move_to_joint_positions(self, joint_pose):
        """"Move arm to a set of joint angles.

        Parameters
        ----------
        joint_pose : Float Array
            The joint position in [rad] corresponding to each joint from base to wrist.
        """

        self.move_success = False

        # Wait until we have received joint states
        while self.latest_joint_state is None:
            return
        
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
            joint_constraint.position = joint_pose[i]
            joint_constraint.tolerance_above = 0.0001  # radians
            joint_constraint.tolerance_below = 0.0001  # radians
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
        self.get_logger().info("Sending goal to move joints")
        # self.get_logger().info(f'Sending goal to move joints {self.joint_names} to positions {joint_pose}')
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        self.sent_goal = True
        
        # Add callbacks for goal response and result
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return 
    
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
            self.move_success = True

        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')
            self.move_success = False
        self.get_logger().info(f"Latest joint states: {self.latest_joint_state.position}")

    def make_square(self, direction):
        """This function sends a goal from the square_dict dictionary.

        Parameters
        ----------
        direction : "right", "left", "up", "down"
            The direction to move in the square.
        """
        self.move_success = False
        
        self.get_logger().info(f"sent: {direction}")
        goal_msg = RelativeMove.Goal()
        goal_msg.relative_pose = self.square_dict[direction]
        send_goal_future = self.relative_move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True

    def gripper_control(self, close=True):
        """ This function sends a goal to the gripper control action server.

        Parameters
        ----------
        close : bool
            Whether to close the gripper (True) or open it (False).
        """
        self.move_success = False

        self.get_logger().info(f"sent gripper command close = {close}")
        goal_msg = GripperControl.Goal()
        goal_msg.close = close
        send_goal_future = self.gripper_control_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True
    
    def set_ground_plane(self):
        """Service call to set the ground plane in aruco detector node.

        Returns
        -------
        future.result : Service Response
            The response from the service call.
        """
        self.get_logger().info("Setting ground plane...")
        request = Trigger.Request()
        future = self.set_plane_cli.call_async(request)
        return future.result()

    def move_to_absolute_pose(self, pose):
        """Sends a goal to absolute move action server.

        Parameters
        ----------
        pose : PoseStamped
            The pose to move the EE in the base frame.
        """
        self.move_success = False

        self.get_logger().info(f"sent pose")
        goal_msg = RelativeMove.Goal()
        goal_msg.relative_pose = pose
        send_goal_future = self.absolute_move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True

    def move_to_relative_pose(self, pose):
        """Sends a goal to the relative move action server.

        Parameters
        ----------
        pose : PoseStamped
            The pose to move the EE relative to current pose.
        """
        self.move_success = False

        self.get_logger().info(f"sent pose")
        goal_msg = RelativeMove.Goal()
        goal_msg.relative_pose = pose
        send_goal_future = self.relative_move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True
    
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
        pose.header.frame_id = "rover_arm_base_link"
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
    
    def get_object_from_intersection(self):
        """Determine the object the user is pointing at. 

        This function uses the intersection of an ArUco markers x-axis and the "ground" plane to find the 
        closest object. The closes object must be within a set radius and the marker (intersection point) 
        must not be moving too much (ie small standard deviation over the last 10 points). The function 
        stores the pick height of the object for later use.

        Returns
        -------
        best_point : PoseStamped
            The pose of the selected object.
        """
        #return if not enough points or point has moved too much
        if len(self.points) < 10 or np.any(self.points_std[0:2] > self.deviation_threshold) or self.object_list == []:
            return None
        best_dist = self.within_radius
        best_point = None
        for point in self.object_list:
            dist = np.linalg.norm(
                np.array([point.pose.position.x, point.pose.position.y])
                - np.array([self.points_mean[0], self.points_mean[1]])
            )
            if dist <= best_dist:
                best_point = point
                # best_point.pose.position.z += self.gripper_offset
        if best_point != None:
            self.pick_height = best_point.pose.position.z
        return best_point
    
    def get_location_from_intersection(self):
        """Find the place location from ground intersection point.

        This function returns the mean of the last 10 intersection points if they are stable enough (ie small 
        standard deviation). Unlike the get_object_from_intersection function, this function is only finding 
        a location not an object. 

        Returns
        -------
        best_point : PoseStamped
            The pose of the selected place location.
        """
        if len(self.points) < 10 or np.any(self.points_std[0:2] > self.deviation_threshold):
            return None
        return self.make_posestamped([self.points_mean[0], self.points_mean[1], self.pick_height, 0.7071068, 0.7071068, 0.0, 0.0])
    
    def get_EE_pose(self):
        """Get the current EE pose.

        Returns
        -------
        current_pose : PoseStamped
            The current EE pose.
        """
        current_pose = PoseStamped()
        try:
            trans = self.tf_buffer.lookup_transform(
                "rover_arm_base_link",  # target frame
                "rover_arm_gripper",  # source frame
                rclpy.time.Time())  # latest available

            current_pose.header = trans.header
            current_pose.pose.position.x = trans.transform.translation.x
            current_pose.pose.position.y = trans.transform.translation.y
            current_pose.pose.position.z = trans.transform.translation.z
            current_pose.pose.orientation = trans.transform.rotation

            # Now you have the current gripper pose
            #self.get_logger().info(f"Gripper pose: {self.current_pose.pose}")
        except Exception as e:
            self.get_logger().error(f"Failed to lookup gripper pose: {str(e)}")
        return current_pose

    def timer_callback(self): 
        """The main control loop for the pick and place routine.

        Notes
        -----
        The current routine uses moveit trajectory planner for all movements do to servo drift (12/16/2025).

        Object detection finds the centroid of the top of the object. Does not fit a shape or determine grasp
        orientation. This cannot be used for collision detection when moving around the objects or placing. 
        """
        #step 1 move to scan position
        if self.state == "start":
            #self.get_logger().info(f"{self.state}")
            start_scan_pose = [
                -0.54105207, 
                -1.08210414,  
                -0.80285146, 
                -0.0,
                -1.23918377, 
                0.54105207
            ]

            #start_scan_pose = [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                #self.get_logger().info("goal")
                time.sleep(0.5)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(start_scan_pose)
            elif self.move_success:
                self.get_logger().info("goto scan")
                self.reset_count_object(enable=True, reset=True)
                self.set_ground_plane()
                self.state = "home"
                # pose = self.get_EE_pose()
                # self.get_logger().info(f"EE Pose: position=({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}), "
                #                     f"orientation=({pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, "
                #                     f"{pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f})")
                
                self.sent_goal = False

        #step 2 scan workspace
        if self.state == "scan":
            # self.start_servo()
            # self.switch_controller(servo=True)

            # #Draw a square
            # self.set_direction()

            # msg = TwistStamped()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.header.frame_id = self.frame_id
            # msg.twist = self.construct_twist()
            # self.configure_servo(self.frame_id)

            # self.publisher_.publish(msg)

            # if self.cycles >= 1:
            #     self.state = "move_to_input"
            #     self.cycles = 0

            # self.start_servo()
            # self.switch_controller(servo=True)

            #make square
            if not self.sent_goal:
                time.sleep(0.5)
                self.make_square(self.sq_keys[self.scan_iter])
            elif self.move_success:
                self.get_logger().info("next scan movement")
                self.sent_goal = False
                self.scan_iter += 1
                if self.scan_iter > 3:
                    self.get_logger().info("goto user input")
                    self.scan_iter = 0
                    self.state = "move_to_input"
                    self.reset_count_object(enable=False, reset=False)

        #step 3 move to get user input position
        if self.state == "move_to_input":
            input_pos =  [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0] #front facing [0.0, -0.34, -1.98968, 0.0, 0.785398, 0.0]# stow [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# Ground pick up: [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                time.sleep(0.5)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.get_logger().info("goto get pick input")
                self.state = "home"
                self.points.clear()
                self.points_mean = np.zeros(3)
                self.points_std = 100 * np.ones(3)
                self.sent_goal = False

        #step 4 get pick input
        if self.state == "get_pick_input":
            # self.get_logger().info("goto object")

            if self.object_list == []:
                self.state = "home"
            else:
                self.object_pose = self.get_object_from_intersection()
                if self.object_pose != None:
                    self.get_logger().info("Goto Move to object")
                    self.state = "move_to_object"
                    self.get_logger().info(f"object pose {self.object_pose}")

        #step 4.5 move to movement position
        if self.state == "move_to_pick_mobility":
            input_pos =  [0.0, -0.34, -1.98968, 0.0, 0.785398, 0.0]# stow [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# Ground pick up: [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                time.sleep(1.0)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.get_logger().info("goto move to object")
                self.state = "move_to_object"
                self.sent_goal = False

        #step 5 move to above object
        if self.state == "move_to_object":
            if not self.sent_goal:
                time.sleep(1.00)
                self.get_logger().info(f"Sent this object pose: {self.object_pose}")
                # self.object_pose = self.make_posestamped([0.0, 0.650, 0.304, 0.001, 1.000, 0.008, -0.005])
                self.move_to_absolute_pose(self.object_pose)
            elif self.move_success:
                current_pose = self.get_EE_pose()
                self.get_logger().info(f"Current EE Pose {current_pose.pose}, Commanded Pose {self.object_pose.pose}")
                self.get_logger().info("goto approach")
                self.sent_goal = False
                time.sleep(0.25)
                self.state = "approach"

        #step 6 approach
        if self.state == "approach":
            if not self.sent_goal:
                time.sleep(0.500)
                approach_pose = self.make_posestamped([0.0, 0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0])
                self.move_to_relative_pose(approach_pose)
            elif self.move_success:
                self.get_logger().info("Goto pick")
                self.sent_goal = False
                time.sleep(0.25)
                self.state = "pick"
            
        #step 6.5 pick
        if self.state == "pick":
            if not self.sent_goal:
                time.sleep(0.500)
                self.gripper_control(close=True)
            elif self.move_success:
                self.get_logger().info("Goto move to place input")
                self.sent_goal = False
                time.sleep(0.25)
                self.state = "move_to_place_input"
        #step 7 move to user input position
        if self.state == "move_to_place_input":
            input_pos =  [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0] #front facing [0.0, -0.34, -1.98968, 0.0, 0.785398, 0.0]# stow [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# Ground pick up: [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                time.sleep(0.5)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.get_logger().info("goto get place input")
                self.state = "get_place_input"
                self.points.clear()
                self.points_mean = np.zeros(3)
                self.points_std = 100 * np.ones(3)
                self.sent_goal = False

        #step 8 get place input
        if self.state == "get_place_input":
            self.object_pose = self.get_location_from_intersection()
            if self.object_pose != None:
                self.get_logger().info("goto place location")
                self.state = "move_above_place_location"
        
        #step 8.5 move to place mobility
        if self.state == "move_to_place_mobility":
            input_pos =  [0.0, -0.34, -1.98968, 0.0, 0.785398, 0.0]# stow [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# Ground pick up: [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                time.sleep(0.5)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.get_logger().info("goto move above place location")
                self.state = "move_above_place_location"
                self.sent_goal = False

        #step 9 move to above place
        if self.state == "move_above_place_location":
            if not self.sent_goal:
                time.sleep(0.500)
                self.get_logger().info(f"Above place location {self.object_pose.pose.position}")
                #   self.object_pose = self.make_posestamped([0.0, 0.650, 0.304, 0.001, 1.000, 0.008, -0.005])
                self.move_to_absolute_pose(self.object_pose)
            elif self.move_success:
                current_pose = self.get_EE_pose()
                self.get_logger().info(f"Current EE Pose {current_pose.pose}, Commanded Pose {self.object_pose.pose}")
                self.get_logger().info("goto place")
                self.sent_goal = False
                time.sleep(0.25)
                self.state = "place_approach"
        
        #step 10 place object
        if self.state == "place_approach":
            if not self.sent_goal:
                time.sleep(0.500)
                approach_pose = self.make_posestamped([0.0, 0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0])
                self.move_to_relative_pose(approach_pose)
            elif self.move_success:
                self.get_logger().info("Goto place")
                self.sent_goal = False
                time.sleep(0.25)
                self.state = "place"

        #step 10.5 place
        if self.state == "place":
            if not self.sent_goal:
                time.sleep(0.500)
                self.gripper_control(close=False)
            elif self.move_success:
                self.get_logger().info("goto sudo home position")
                self.sent_goal = False
                time.sleep(0.25)
                self.state = "sudo_home"

        #step 11 Return to user input
        if self.state == "sudo_home":
            input_pos =  [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# stow [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# Ground pick up: [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                time.sleep(0.5)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.get_logger().info("Done")
                self.state = "start"
                self.sent_goal = False

        #step 12 Return to Home sometimes
        if self.state == "home":
            input_pos =  [0.0, 0.0, -0.0, 0.0, 0.0, 0.0]# stow [0.17453, 1.22173, -2.61799, 0.0, -0.17453, 0.0]# Ground pick up: [0.0, -0.698132, -1.65806, 0.0, -0.785698, 0.0]
            if not self.sent_goal:
                time.sleep(0.5)
                self.switch_controller(servo=False)
                self.move_to_joint_positions(input_pos)
            if self.move_success:
                self.get_logger().info("Done")
                self.state = "Next"
                self.sent_goal = False

    def set_direction(self):

        if self.loops == self.dir_time[self.curr_dir]:
            self.loops = 0
            prev_dir = self.curr_dir
            self.curr_dir = self.sequence[self.curr_dir]
            if self.curr_dir == "down" and prev_dir == "left":
                self.cycles += 1
            self.get_logger().info("Going {}!".format(self.curr_dir))

        self.loops = self.loops + 1

    def construct_twist(self):
        
        twist = Twist()
        vals = self.dir_def[self.curr_dir]

        twist.linear.x = vals[0]
        twist.linear.y = vals[1]

        return twist

def main(args=None):
    rclpy.init(args=args)

    pick_and_place = SquareMakingController()

    # while rclpy.ok():
    #     rclpy.spin_once(pick_and_place)
    rclpy.spin(pick_and_place)

    # executor = MultiThreadedExecutor()
    # executor.add_node(pick_and_place)
    # executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pick_and_place.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()