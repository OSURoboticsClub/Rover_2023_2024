from enum import Enum, auto
from std_msgs.msg import Float32, String
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
    SUCCESS = 4
    FAILED = 5
    STOPPED = 6


class SearchFSM:
    def __init__(self, node, navigator, confidence_topic, status_topic):

        self.node = node
        self.navigator = navigator

        # Mission config
        self.start_point = None
        self.pattern = None
        self.success_threshold = 1.0
        self.investigate_threshold = 1.0

        # State
        self.state = SearchState.IDLE
        self.active = False
        self.search_path = []
        self.current_index = 0
        self.best_conf = 0.0
        self.best_pose = None

        # Pub/Sub
        self.status_pub = node.create_publisher(String, status_topic, 10)
        self.conf_sub = node.create_subscription(Float32, confidence_topic, self._confidence_cb, 10)


        self.timer = node.create_timer(0.2, self._tick)


    def start(self, start_path, search_points, pattern, success_threshold, investigate_threshold):
        self.start_point = search_points[0]
        self.success_threshold = success_threshold
        self.investigate_threshold = investigate_threshold
        self.best_conf = 0.0
        self.best_pose = None
        self.current_index = 0

        self.search_path = []
        self.pattern = pattern
        match self.pattern:
            case SearchPattern.SPIRAL:
                self.search_path = spiral(center=self.start_point, max_radius=10.0, spacing=2.0, points_per_revolution=12)
            case SearchPattern.LAWNMOWER:
                self.search_path = lawnmower(corners=search_points, spacing=2.0, step_size=1.0)
            case _:
                self.node.get_logger().error(f'Unknown search pattern: {self.pattern}. Cannot start search.')
                return
        
        self.path_to_start = start_path  #TODO: handle getting to search start_points
        self.state = SearchState.MOVING_TO_START
        self.active = True
        self._publish()


    def stop(self):
        self.navigator.cancelTask()
        self.state = SearchState.STOPPED
        self.active = False
        self._publish()


    def reset(self):
        self.stop()
        self.state = SearchState.IDLE
        self._publish()


    def get_state(self):
        return self.state

    def is_active(self):
        return self.active

    def is_done(self):
        return self.state in (SearchState.SUCCESS, SearchState.FAILED, SearchState.STOPPED)


    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def _confidence_cb(self, msg):
        if not self.active:
            return

        conf = msg.data

        if conf > self.best_conf:
            self.best_conf = conf
            self.best_pose = self._current_pose()

        # Maybe want to adjust speed based on confidence??
        if 0.7 < conf < self.success_threshold:
            self.navigator.setSpeedLimit(0.3, is_percentage=True)
        else:
            self.navigator.setSpeedLimit(1.0, is_percentage=True)

        # Trigger success or investigation
        if conf >= self.success_threshold:
            self._to_success()
        elif conf >= self.investigate_threshold:
            if self.state == SearchState.SEARCHING:
                self._to_investigate()

    # --------------------------------------------------
    # FSM Tick
    # --------------------------------------------------

    def _tick(self):

        if not self.active:
            return

        nav_feedback = self.navigator.getFeedback()
        if nav_feedback:
            self.current_index = max(0, nav_feedback.current_waypoint_index - 1)
            #TODO: publish more nav feedback

        if self.state == SearchState.MOVING_TO_START:
            self._state_move_to_start()

        elif self.state == SearchState.SEARCHING:
            self._state_searching()

        elif self.state == SearchState.INVESTIGATING:
            self._state_investigating()

        self._publish()


    # --------------------------------------------------
    # States
    # --------------------------------------------------

    def _state_move_to_start(self):

        if not self.navigator.isTaskComplete():
            return

        self.navigator.followWaypoints(self.path_to_start)


    def _state_searching(self):

        if not self.navigator.isTaskComplete():
            return

        if self.best_conf < self.success_threshold:
            # if self.best_pose is not None:
            #     self._restart_near_best()
            # else:
            self._to_failed()
        else:
            self._to_success()


    def _state_investigating(self):

        if not self.navigator.isTaskComplete():
            return

        # After investigate, resume search from where we left off
        self.navigator.followWaypoints(self.search_path[self.current_index:])
        self.state = SearchState.SEARCHING


    # --------------------------------------------------
    # Transitions
    # --------------------------------------------------

    def _to_success(self):
        self.navigator.cancelTask()
        self.state = SearchState.SUCCESS
        self.active = False
        self._publish()


    def _to_failed(self):
        self.state = SearchState.FAILED
        self.active = False
        self._publish()


    def _to_investigate(self):
        self.navigator.cancelTask()

        feedback = self.navigator.getFeedback()
        if feedback:
            self.resume_index = feedback.current_waypoint_index
        self.navigator.goToPose(self.best_pose)
        self.state = SearchState.INVESTIGATING
        self._publish()


    # def _restart_near_best(self):
    #     self.start_point = self.best_pose
    #     self.search_path = search_patterns.generate_lawnmower(
    #         start_point=self.start_point,
    #         width=10.0,
    #         height=10.0,
    #         spacing=2.0,
    #         step_size=1.0
    #     )
    #     self.navigator.followWaypoints(self.search_path)
    #     self.state = SearchState.SEARCHING
    #     self._publish()


    # --------------------------------------------------
    # Utilities
    # --------------------------------------------------

    def _current_pose(self):

        if self.current_index >= len(self.search_path):
            return None
        return self.search_path[self.current_index]


    def _publish(self):

        msg = String()
        msg.data = self.state.name
        # TODO: publish path + current index, nav feedback, best pose, etc.
        self.status_pub.publish(msg)


