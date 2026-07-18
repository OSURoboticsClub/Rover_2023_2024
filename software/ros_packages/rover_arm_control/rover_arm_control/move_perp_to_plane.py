"""
Move Perpendicular to Plane Node
DAM Robotics
Authors: Jared Northrop
Year: 2526

This node sends a move request to moveit 2 based on the most prominant plane in the point cloud. 
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
from shape_msgs.msg import Plane

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
from scipy.spatial.transform import Rotation


class PerpToPlane(Node):

    def __init__(self):
        super().__init__('perp_to_plane')
        self.get_clock().now()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cb_group = MutuallyExclusiveCallbackGroup()

        #Servers
        self.move_perp_server = self.create_service(Trigger, 'move_perp_to_plane', self.move_perp_callback)

        #clients
        self.absolute_move_client = ActionClient(self, RelativeMove, 'absolute_move')
        self.get_logger().info('Waiting for AbsoluteMove action server...')
        self.absolute_move_client.wait_for_server()
        self.get_logger().info('AbsoluteMove action server connected!')

        #Publishers
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)

        #subscribers
        self.plane_sub = self.create_subscription(
            Plane,
            '/ground_plane',
            self.plane_callback,
            10
        )

        #state params
        self.sent_goal = False
        self.plane_history = deque(maxlen=10)
        self.plane_coeff_sum = np.zeros(3)
        self.plane_avg = None
        self.current_pose = None
        self.reference_frame = "rover_arm_base_link"
        self.target_frame = "rover_arm_gripper"
    
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

    def find_tool_orientation(self, z):
        """Determine the inward vector perpendicular to the tangent line. 

        This function is the main logic in this node. It uses the plane coefficients to determine the z-axis 
        of gripper. It then chooses the x and y on the global x-axis and y-axis. Currently 5/26 it was seen 
        that the gripper gets flipped upsidown. This can probably be fixed by 1. negating the x vector, 2. 
        negating the x_ref vector, or 3. switching the order of the cross product when finding x 
        (i.e., x = np.cross(x_ref, z) -> np.cross(z, x_ref))

        Parameters
        ----------
        z : array
            The tool orientation vector, which is the z axis of the end effector [x, y, z].
        
        Returns
        -------
        quat : array
            The orientation of the tool as a quaternion [x, y, z, w].
        rot : Rotation
            The orientation of the tool as a scipy Rotation object.
        x : array
            The x axis of the end effector in world coordinates as a unit vector.
        y : array
            The y axis of the end effector in world coordinates as a unit vector.
        z : array
            The z axis of the end effector in world coordinates as a unit vector.
        """
        # z direction is the tool orientation
        #get the unit vector in the z direction
        z = z / np.linalg.norm(z)
        #Choose an arbitrary x reference
        x_ref = np.array([0.0, 0.0, 1.0])
        #Check if x reference is parallel to z
        if np.fabs(z.dot(x_ref)) > 0.99:
            x_ref = np.array([0.0, 1.0, 0.0])
        #Find the perpendicular vector and set to be the x axis
        x = np.cross(z, x_ref)
        x = x / np.linalg.norm(x)
        #Find the y axis
        y = np.cross(z, x)
        #Create rotation matrix
        R = np.column_stack((x, y, z))
        rot = Rotation.from_matrix(R)
        #convert rotation matrix to quaternion
        quat = rot.as_quat()
        return quat, rot, x, y, z
    
    def plane_callback(self, msg):
        # Keep a running average of plane coefficients
        plane = np.array([msg.coef[0], msg.coef[1], msg.coef[2]])
        # self.get_logger().info(f"Received plane coefficients: {plane}")

        if len(self.plane_history) == self.plane_history.maxlen:
            oldest = self.plane_history[0]
            self.plane_coeff_sum -= oldest

        self.plane_history.append(plane)
        self.plane_coeff_sum += plane

        self.plane_avg = self.plane_coeff_sum / len(self.plane_history)

    def get_EE_pose(self):
        # Get the current pose of the gripper
        try:
            trans = self.tf_buffer.lookup_transform(
                self.reference_frame,  # target frame
                self.target_frame,  # source frame
                rclpy.time.Time())  # latest available

            self.current_pose = PoseStamped()
            self.current_pose.header = trans.header
            self.current_pose.pose.position.x = trans.transform.translation.x
            self.current_pose.pose.position.y = trans.transform.translation.y
            self.current_pose.pose.position.z = trans.transform.translation.z
            self.current_pose.pose.orientation = trans.transform.rotation

        except Exception as e:
            self.current_pose = None
            self.get_logger().error(f"Failed to lookup gripper pose: {str(e)}")

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
        self.sent_goal = False

    def move_perp_callback(self, request, response):
        # Send a move request to moveit 2 based on the most prominant plane in the point cloud.
        if self.sent_goal:
            self.get_logger().info("A request is already in progress.")
            response.success = False
            response.message = "A request is already in progress."
            return response
        if self.plane_avg is None:
            response.success = False
            response.message = "No plane data received yet."
            return response
        self.get_logger().info(f"Moving perpendicular to plane with average coefficients: {self.plane_avg}")
        self.get_EE_pose()
        if self.current_pose is None:
            response.success = False
            response.message = "Could not get current end effector pose."
            return response
        quat, rot, x, y, z = self.find_tool_orientation(self.plane_avg)
        self.get_logger().info(f"Calculated tool orientation: quat={quat}, x={x}, y={y}, z={z}")
        # Create a goal message for the absolute_move action
        goal_msg = RelativeMove.Goal()
        goal_msg.relative_pose = self.current_pose
        goal_msg.relative_pose.pose.orientation.x = quat[0]
        goal_msg.relative_pose.pose.orientation.y = quat[1]
        goal_msg.relative_pose.pose.orientation.z = quat[2]
        goal_msg.relative_pose.pose.orientation.w = quat[3]

        # Send the goal to the absolute_move action server
        self.get_logger().info("Sending goal to absolute_move action server...")
        send_goal_future = self.absolute_move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True
        response.success = True
        response.message = "Move command sent to absolute_move action server."
        return response
    


def main(args=None):
    rclpy.init(args=args)

    perp_to_plane = PerpToPlane()

    # while rclpy.ok():
    #     rclpy.spin_once(pick_and_place)
    rclpy.spin(perp_to_plane)

    # executor = MultiThreadedExecutor()
    # executor.add_node(perp_to_plane)
    # executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    perp_to_plane.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()