from enum import Enum, auto
from typing import List
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from nav_autonomy.utils.search_patterns import spiral, lawnmower

class SearchPattern(Enum):
    NONE = auto()
    SPIRAL = auto()
    LAWNMOWER = auto()


class SearchState(Enum):
    IDLE = 0
    MOVING_TO_START = 1
    SEARCHING = 2
    INVESTIGATING = 3
    RETURNING_TO_SEARCH = 4
    SUCCESS = 5
    FAILED = 6
    STOPPED = 7


class SearchFSM:
    def __init__(self, node, navigator, confidence_topic, status_topic):

        self.node = node
        self.navigator = navigator

        self.pattern = SearchPattern.NONE
        self.success_threshold = 0.85
        self.investigate_threshold = 0.50

        self.state = SearchState.IDLE
        self.active = False
        
        self.active_path: List[PoseStamped] = []
        self.resume_path: List[PoseStamped] = []
        self.start_length = 0
        self.current_index = 0

        self.status_pub = node.create_publisher(String, status_topic, 10)
        self.conf_sub = node.create_subscription(Float32, confidence_topic, self._confidence_cb, 10)

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
        self._publish_state()

    def stop(self):
        self.navigator.cancelTask()
        self.active = False
        self.state = SearchState.STOPPED
        self._publish_state()

    def tick(self):
        if not self.active:
            return

        # Currently navigating to waypoints, check for completion or progress
        # FIX: Might have used non-Humble version of the followwaypoints API, see: https://api.nav2.org/actions/humble/followwaypoints.html
        if not self.navigator.isTaskComplete():
            nav_feedback = self.navigator.getFeedback()
            if nav_feedback:
                #FIX: followWapoints and goToPose feedback is different, see: https://api.nav2.org/actions/humble/navigatetopose.html
                self.current_index = nav_feedback.current_waypoint
                
                if self.state == SearchState.MOVING_TO_START and self.current_index >= self.start_length:
                    self.state = SearchState.SEARCHING
                    self._publish_state()

        else:
            # waypoint task completed
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if self.state in (SearchState.SEARCHING, SearchState.MOVING_TO_START):
                    self._to_failed() 
                elif self.state == SearchState.INVESTIGATING:
                    self._return_to_search()
                elif self.state == SearchState.RETURNING_TO_SEARCH:
                    self.active_path = self.resume_path
                    self.start_length = 0 
                    if self.active_path:
                        self.navigator.followWaypoints(self.active_path)
                        self.state = SearchState.SEARCHING
                        self._publish_state()
                    else:
                        self._to_failed()
            elif result == TaskResult.CANCELED:
                pass 
            else:
                self._to_failed()

    def _confidence_cb(self, msg: Float32):
        if not self.active:
            return

        conf = msg.data

        # TODO: Consider some throttling or smoothing to avoid rapid state changes on false positives/negatives

        if self.state == SearchState.SEARCHING and conf >= self.investigate_threshold:
            self._to_investigate()
            return

        if self.state == SearchState.INVESTIGATING and conf >= self.success_threshold:
            self.state = SearchState.SUCCESS
            self.active = False
            self.navigator.cancelTask()
            self._publish_state()

    def _to_investigate(self):
        self.navigator.cancelTask()

        resume_index = max(0, self.current_index - 1)
        self.resume_path = self.active_path[resume_index:]
        
        self.state = SearchState.INVESTIGATING
        self._publish_state()
        
        # TODO: Execute investigation behavior

    def _return_to_search(self):
        self.state = SearchState.RETURNING_TO_SEARCH
        self._publish_state()
        
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

    def get_state(self):
        return self.state

    def is_active(self):
        return self.active
