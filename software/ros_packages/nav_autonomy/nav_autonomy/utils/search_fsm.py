from enum import Enum, auto
from typing import List
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from nav_autonomy.utils.search_patterns import spiral, lawnmower
from visualization_msgs.msg import Marker, MarkerArray
import time

class SearchPattern(Enum):
    NONE = auto()
    SPIRAL = auto()
    LAWNMOWER = auto()


class SearchState(Enum):
    IDLE = auto()
    MOVING_TO_START = auto()
    SEARCHING = auto()
    INVESTIGATION_PENDING = auto()
    INVESTIGATING = auto()
    RETURNING_TO_SEARCH = auto()
    SUCCESS = auto()
    FAILED = auto()
    STOPPED = auto()


class SearchFSM:
    def __init__(self, node, navigator, investigate_threshold = 0.5, success_threshold = 0.85, status_topic='search/status', debug_markers=False, dbg_marker_topic="search/debug_markers"):

        self.node = node
        self.navigator = navigator

        self.state = SearchState.IDLE
        self.active = False
        
        self.pattern = SearchPattern.NONE
        self.active_path: List[PoseStamped] = []
        self.resume_path: List[PoseStamped] = []
        self.start_length = 0
        self.current_index = 0

        self.investigate_threshold = investigate_threshold
        self.success_threshold = success_threshold
        # self.investigate_theshold_met = False
        self.target_pose = None
        self.last_detection_time = None
        self.detection_timeout_ms = 8000.0
        
        self.status_pub = node.create_publisher(String, status_topic, 10)
        self.debug_marker_pub = node.create_publisher(MarkerArray, dbg_marker_topic, 10) if debug_markers else None


    def start(self, start_path: List[PoseStamped], search_inputs: List[PoseStamped], pattern: SearchPattern, param1: float, param2: float, success_threshold: float = 0.85, investigate_threshold: float = 0.50):

        self.success_threshold = success_threshold
        self.investigate_threshold = investigate_threshold
        
        self.pattern = pattern
        pattern_path = []

        if self.pattern == SearchPattern.SPIRAL and len(search_inputs) == 1:
            pattern_path = spiral(center=search_inputs[0], spacing=param1, max_radius=param2)
        elif self.pattern == SearchPattern.LAWNMOWER and len(search_inputs) >= 4:
            pattern_path = lawnmower(corners=search_inputs, spacing=param1, step_size=param2)

        self.active_path = start_path + pattern_path
        self.start_length = len(start_path)
        self.current_index = 0
        self.state = SearchState.MOVING_TO_START
        self.active = True

        if not self.active_path:
            self._to_failed()
            return

        self.navigator.followWaypoints(self.active_path)
        # self.navigator.goThroughPoses(self.active_path)
        self._publish_state()
        self._publish_dbg_waypoint_markers()


    def stop(self):
        self.navigator.cancelTask()
        self.active = False
        self.state = SearchState.STOPPED
        self._publish_state()


    def tick(self):
        if not self.active:
            return

        # spin until cancelTask is complete then start investigation
        if self.state == SearchState.INVESTIGATION_PENDING:
        # if self.investigate_theshold_met:
            self.node.get_logger().info(f"We are investigate pending ONCE")
            if self.navigator.isTaskComplete():
                self._to_investigate()
            return

        # If we've been investigating for too long without a yolo detection, return to search
        if self.state == SearchState.INVESTIGATING and self.last_detection_time is not None:
            time_since_detection = (self.node.get_clock().now() - self.last_detection_time).nanoseconds / 1e6 # convert to milliseconds
            if time_since_detection > self.detection_timeout_ms:
                self.node.get_logger().warn(f"INVESTIGATION: Haven't seen target in {time_since_detection:.1f}, returning to search.")
                self.navigator.cancelTask()
                self._return_to_search()
                return
        
        if self.navigator.isTaskComplete():
            self._process_nav_result()
        else:
            self._process_nav_feedback()


    def update_perception(self, confidence: float, target_pose: PoseStamped):
        if not self.active:
            return

        if target_pose is None:
            self.node.get_logger().warning(f"Perception update: no target pose, how'd that happen?")
            return

        
        self.node.get_logger().info(f"[Perception]: Updating confidence ({confidence})")
        self.target_pose = target_pose
        self.last_detection_time = self.node.get_clock().now()

        self._publish_dbg_waypoint_markers()

        if self.state == SearchState.SEARCHING and confidence >= self.investigate_threshold:
            self.state = SearchState.INVESTIGATION_PENDING
            # self.investigate_theshold_met = True
            self.navigator.cancelTask()
            #self._to_investigate()
            return
        
        # WINNER: Object found, end everything.
        if self.state == SearchState.INVESTIGATING and confidence >= self.success_threshold:
            self.state = SearchState.SUCCESS
            self.active = False
            self.navigator.cancelTask()
            self._publish_state()   


    def get_state(self):
        return self.state


    def is_active(self):
        return self.active
    

    def _process_nav_feedback(self):
        nav_feedback = self.navigator.getFeedback()
        if not nav_feedback:
            return

        match self.state:
            case SearchState.MOVING_TO_START | SearchState.SEARCHING | SearchState.RETURNING_TO_SEARCH:
                # Handle followWaypoints feedback
                if hasattr(nav_feedback, 'current_waypoint'):
                    self.current_index = nav_feedback.current_waypoint
                # Handle goThroughPoses feedback
                # if hasattr(nav_feedback, 'number_of_poses_remaining'):
                #     self.current_index = len(self.active_path) - nav_feedback.number_of_poses_remaining
                
                # State transition when reaching the start of the pattern
                if self.state == SearchState.MOVING_TO_START and self.current_index >= self.start_length:
                    self.state = SearchState.SEARCHING
                    self._publish_state()

            case SearchState.INVESTIGATING:
                # Handle goToPose feedback
                if hasattr(nav_feedback, 'distance_remaining'):
                    self.node.get_logger().info(f"Investigate: Dist to target: {nav_feedback.distance_remaining}")
                    pass


    def _process_nav_result(self):
        result = self.navigator.getResult()
        self.node.get_logger().info(f"Navigation task completed with result: {result}")

        # If navigation failed and wasn't canceled, something went wrong, end the search with failure. 
        # If it was canceled, it's likely due to an investigation interrupt or return to search, so just wait for the next command.
        if result != TaskResult.SUCCEEDED:
            self.node.get_logger().info(f"Didn't succeed, checking if canceled: {result}")
            # if result == TaskResult.FAILED:
            #     self._to_investigate()

            if result != TaskResult.CANCELED:
                self._to_failed()
                
            return

        # Nav arrived at goal
        match self.state:
            
            # Arrived at the end of the search space without hitting success thresholds
            case SearchState.MOVING_TO_START | SearchState.SEARCHING:
                self._to_failed()
                
            # Arrived at suspected object's location but YOLO didn't confirm it
            case SearchState.INVESTIGATING:
                self.node.get_logger().info("Reached investigation coordinates, but threshold not met.")
                self._return_to_search()
                
            # Arrived back to the search breakpoint, resume the search
            case SearchState.RETURNING_TO_SEARCH:
                if len(self.resume_path) > 1:
                    self.active_path = self.resume_path[1:] # drop the first pose since we're already there
                else:
                    self.active_path = []
                self.start_length = 0 
                if self.active_path:
                    self.navigator.followWaypoints(self.active_path)
                    # self.navigator.goThroughPoses(self.active_path)
                    self.state = SearchState.SEARCHING
                    self._publish_state()
                else:
                    self._to_failed()


    def _to_investigate(self):
        #self.navigator.cancelTask()
        # self.investigate_theshold_met = False

        resume_index = max(0, self.current_index - 1)
        self.resume_path = self.active_path[resume_index:]

        self.state = SearchState.INVESTIGATING
        self._publish_state()
        
        self.node.get_logger().info(f"Interrupt Triggered: Investigating target at X: {self.target_pose.pose.position.x:.2f}, Y: {self.target_pose.pose.position.y:.2f}")
        self.node.get_logger().info(f"big sleeping")
        time.sleep(1)
        self.node.get_logger().info(f"Done big sleeping")
        if not self.navigator.goToPose(self.target_pose):
            self.node.get_logger().error("Failed to accept goToPose for investigation.")
            self._to_failed()
        else:
            self.node.get_logger().warn("Accepted Investigate Point")


    def _return_to_search(self):
        self.state = SearchState.RETURNING_TO_SEARCH
        self.target_pose = None
        self.last_detection_time = None

        self._publish_state()

        self.node.get_logger().info(f"big sleeping")
        time.sleep(1)
        self.node.get_logger().info(f"Done big sleeping")
        if self.resume_path:
            self.navigator.goToPose(self.resume_path[0])
        else:
            self._to_failed()


    def _to_failed(self):
        self.state = SearchState.FAILED
        self.active = False
        self._publish_state()


    def _publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.status_pub.publish(msg)
        self.node.get_logger().info(f"[SearchFSM] State: {self.state.name}")


    def _publish_dbg_waypoint_markers(self):
        if self.debug_marker_pub is None:
            return

        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, wp in enumerate(self.active_path):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'mission_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.pose.position.z = 0.25 
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            
            r, g, b, a = 0.45, 0.73, 0.98, 0.8
            if i >= self.start_length:
                r, g, b = 0.98, 0.92, 0.55
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            marker_array.markers.append(marker)

        # post current target
        if self.target_pose is not None:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = 'mission_waypoints'
            marker.id = 100
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = self.target_pose.pose
            marker.pose.position.z = 0.25 
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            
            r, g, b, a = 0.9, 0.4, 0.0, 0.8
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            marker_array.markers.append(marker)

        self.debug_marker_pub.publish(marker_array)
