#!/usr/bin/python3

from time import time
import math
import struct

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool


AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0b
CMD_SET_INPUT_VEL = 0x0d
CMD_GET_IQ = 0x14
CMD_CLEAR_ERRORS = 0x18

CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_VEL_RAMP = 2


class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__("linear_actuator_control")
        # Both of these need to be replaced, they do not need to be parameters
        self.declare_parameter("can", "can0")
        # check the node id
        self.declare_parameter("node_id", 1)
        self.declare_parameter("command_topic", "linear_actuator/control")
        self.declare_parameter("position_topic", "linear_actuator/position")
        self.declare_parameter("current_topic", "linear_actuator/current")
        # check if this is needed
        self.declare_parameter("failsafe_topic", "linear_actuator/failsafe_active")
        self.declare_parameter("command_timeout_s", 1.0)
        # 20 hz
        self.declare_parameter("timer_period_s", 0.05)
        self.declare_parameter("telemetry_request_period_s", 0.05)
        # this should be -1 if going backwards? review needed
        self.declare_parameter("velocity_scale", 1.0)
        # this is obviously going to be higher - 11400 maximum rotations theoretically
        self.declare_parameter("velocity_limit_rps", 300)
        # this is also faster
        self.declare_parameter("input_vel_ramp", 200.0)
        # what is the current limit?
        self.declare_parameter("current_limit_amps", 0.0)
        self.declare_parameter("idle_on_failsafe", True)
        self.declare_parameter("home_current_threshold_amps", 5.0)
        self.declare_parameter("home_velocity_rps", -40.0)
        self.declare_parameter("home_current_samples", 5)
        self.declare_parameter("home_velocity_cutoff_ratio", 0.7)
        self.declare_parameter("home_velocity_reach_ratio", 0.9)
        self.declare_parameter("home_velocity_cutoff_samples", 3)
        self.declare_parameter("home_rampup_margin_s", 0.1)
        self.declare_parameter("home_backoff_rotations", 20.0)
        self.declare_parameter("home_backoff_velocity_rps", 40.0)
        self.declare_parameter("min_position_rotations", 0.0)
        self.declare_parameter("max_position_rotations", 7300.0)
#!/usr/bin/python3

from time import time
import math
import struct

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool


AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0b
CMD_SET_INPUT_VEL = 0x0d
CMD_GET_IQ = 0x14
CMD_CLEAR_ERRORS = 0x18

CONTROL_MODE_VELOCITY_CONTROL = 2
INPUT_MODE_VEL_RAMP = 2


class LinearActuatorControl(Node):
    def __init__(self):
        super().__init__("linear_actuator_control")
        # Both of these need to be replaced, they do not need to be parameters
        self.declare_parameter("can", "can0")
        # check the node id
        self.declare_parameter("node_id", 1)
        self.declare_parameter("command_topic", "linear_actuator/control")
        self.declare_parameter("position_topic", "linear_actuator/position")
        self.declare_parameter("current_topic", "linear_actuator/current")
        # check if this is needed
        self.declare_parameter("failsafe_topic", "linear_actuator/failsafe_active")
        self.declare_parameter("command_timeout_s", 1.0)
        # 20 hz
        self.declare_parameter("timer_period_s", 0.05)
        self.declare_parameter("telemetry_request_period_s", 0.05)
        # this should be -1 if going backwards? review needed
        self.declare_parameter("velocity_scale", 1.0)
        # this is obviously going to be higher - 11400 maximum rotations theoretically
        self.declare_parameter("velocity_limit_rps", 300)
        # this is also faster
        self.declare_parameter("input_vel_ramp", 200.0)
        # what is the current limit?
        self.declare_parameter("current_limit_amps", 0.0)
        self.declare_parameter("idle_on_failsafe", True)
        self.declare_parameter("home_current_threshold_amps", 5.0)
        self.declare_parameter("home_velocity_rps", -40.0)
        self.declare_parameter("home_current_samples", 5)
        self.declare_parameter("home_velocity_cutoff_ratio", 0.7)
        self.declare_parameter("home_velocity_reach_ratio", 0.9)
        self.declare_parameter("home_velocity_cutoff_samples", 3)
        self.declare_parameter("home_rampup_margin_s", 0.1)
        self.declare_parameter("home_backoff_rotations", 20.0)
        self.declare_parameter("home_backoff_velocity_rps", 40.0)
        self.declare_parameter("min_position_rotations", 0.0)
        self.declare_parameter("max_position_rotations", 8500.0)

        # can info
        self.can_bus = self.get_parameter("can").value
        self.node_id = int(self.get_parameter("node_id").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)

        self.telemetry_request_period_s = float(
            self.get_parameter("telemetry_request_period_s").value
        )

        # velocity and current limits
        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.velocity_limit_rps = abs(float(self.get_parameter("velocity_limit_rps").value))
        self.input_vel_ramp = abs(float(self.get_parameter("input_vel_ramp").value))
        self.current_limit_amps = abs(float(self.get_parameter("current_limit_amps").value))
        self.idle_on_failsafe = bool(self.get_parameter("idle_on_failsafe").value)
        self.home_current_threshold_amps = abs(
            float(self.get_parameter("home_current_threshold_amps").value)
        )
        self.home_velocity_rps = float(self.get_parameter("home_velocity_rps").value)
        self.home_current_samples = max(
            1, int(self.get_parameter("home_current_samples").value)
        )
        self.home_velocity_cutoff_ratio = max(
            0.0, min(1.0, float(self.get_parameter("home_velocity_cutoff_ratio").value))
        )
        self.home_velocity_reach_ratio = max(
            self.home_velocity_cutoff_ratio,
            min(1.0, float(self.get_parameter("home_velocity_reach_ratio").value)),
        )
        self.home_velocity_cutoff_samples = max(
            1, int(self.get_parameter("home_velocity_cutoff_samples").value)
        )
        self.home_rampup_margin_s = max(
            0.0, float(self.get_parameter("home_rampup_margin_s").value)
        )
        self.home_backoff_rotations = abs(
            float(self.get_parameter("home_backoff_rotations").value)
        )
        self.home_backoff_velocity_rps = abs(
            float(self.get_parameter("home_backoff_velocity_rps").value)
        )
        self.min_position_rotations = float(
            self.get_parameter("min_position_rotations").value
        )
        self.max_position_rotations = float(
            self.get_parameter("max_position_rotations").value
        )
        if self.max_position_rotations < self.min_position_rotations:
            self.min_position_rotations, self.max_position_rotations = (
                self.max_position_rotations,
                self.min_position_rotations,
            )


        self.command_velocity_rps = 0.0
        # part of drill setup is putting it at 0.
        self.position_rev = 0.0
        # defaults
        self.velocity_rps = 0.0
        self.iq_set_amps = 0.0
        self.iq_measured_amps = 0.0
        self.failsafe_active = False
        self.last_command_time = 0.0
        self.last_telemetry_request_time = 0.0
        self.is_homed = True
        self.home_current_count = 0
        self.home_backoff_active = False
        self.home_backoff_target_rev = 0.0
        self.home_start_time = 0.0
        self.home_reached_cruise_speed = False
        self.home_velocity_cutoff_count = 0
        self.home_zero_position_rev = 0.0
        self.assumed_home_set = False

        self.bus = can.interface.Bus(self.can_bus, interface="socketcan")
        self.flush_can_buffer()
        self.setup_controller()

        # Subscription to a node controlling the lin act
        self.command_subscription = self.create_subscription(
            Float32,
            self.get_parameter("command_topic").value,
            self.command_callback,
            10,
        )
        # Position of lin act
        self.position_pub = self.create_publisher(
            Float32,
            self.get_parameter("position_topic").value,
            10,
        )
        # Current of motor
        self.current_pub = self.create_publisher(
            Float32,
            self.get_parameter("current_topic").value,
            10,
        )
        # Failsafe topic and service
        self.failsafe_pub = self.create_publisher(
            Bool,
            self.get_parameter("failsafe_topic").value,
            10,
        )
        self.reset_service = self.create_service(
            SetBool,
            "linear_actuator/reset_failsafe",
            self.reset_failsafe_callback,
        )

        self.timer = self.create_timer(
            float(self.get_parameter("timer_period_s").value),
            self.timer_callback,
        )

        # Set the failsafe or warn if not set.
        # if self.current_limit_amps <= 0.0:
        #     self.get_logger().warn(
        #         "current_limit_amps is 0.0, so the current failsafe is disabled"
        #     )



    def setup_controller(self):
        try:
            self.send_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL) # 8
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_SET_CONTROLLER_MODE),
                    data=struct.pack(
                        "<II",
                        CONTROL_MODE_VELOCITY_CONTROL,
                        INPUT_MODE_VEL_RAMP, # this is 2, 2
                    ),
                    is_extended_id=False,
                )
            )
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(0x04),
                    data=struct.pack("<BHBf", 1, 403, 0, self.input_vel_ramp),
                    is_extended_id=False,
                )
            )
        except Exception as exc:
            self.get_logger().error(f"Linear actuator CAN setup error: {exc}")

    def command_callback(self, msg):
        if self.failsafe_active:
            self.command_velocity_rps = 0.0
            return

        scaled_velocity = float(msg.data) * self.velocity_scale
        self.command_velocity_rps = self.clamp_velocity(scaled_velocity)
        self.last_command_time = time()

    def timer_callback(self):
        self.read_can()
        self.request_telemetry()
        if not self.assumed_home_set:
            self.home_zero_position_rev = self.position_rev
            self.assumed_home_set = True
            # self.get_logger().info(
            #     f"Assumed homed at startup. Zero position set to {self.home_zero_position_rev:.3f} rev"
            # )

        # Homing flow intentionally disabled per updated protocol.
        # This block previously ran automatic homing: drive in reverse until current/velocity
        # cutoff indicated top-of-travel, then move forward by backoff rotations before
        # setting is_homed=True and zeroing position.
        # if not self.is_homed:
        #     self.home_linear_actuator()
        #     self.publish_telemetry()
        #     return

        self.check_current_failsafe()

        if time() >= self.last_command_time + self.command_timeout_s:
            self.command_velocity_rps = 0.0

        if self.failsafe_active:
            self.send_velocity(0.0)
        else:
            self.send_velocity(self.limit_velocity_by_position(self.command_velocity_rps))

        self.publish_telemetry()

    def home_linear_actuator(self):
        if self.failsafe_active:
            return

        if self.home_start_time == 0.0:
            self.home_start_time = time()

        if self.home_backoff_active:
            if self.has_reached_home_backoff_target():
                self.send_velocity(0.0)
                self.is_homed = True
                self.home_zero_position_rev = self.position_rev
                self.command_velocity_rps = 0.0
                self.last_command_time = time()
                self.get_logger().info("Linear actuator homed")
            else:
                self.send_velocity(self.home_backoff_velocity_command())
            return

        if abs(self.iq_measured_amps) >= self.home_current_threshold_amps:
            self.home_current_count += 1
        else:
            self.home_current_count = 0

        current_cutoff_reached = self.home_current_count >= self.home_current_samples
        velocity_cutoff_reached = False
        if self.is_past_home_rampup():
            target_home_speed = abs(self.home_velocity_rps)
            if (
                not self.home_reached_cruise_speed
                and (abs(self.velocity_rps)
                >= target_home_speed * self.home_velocity_reach_ratio)
            ):
                self.home_reached_cruise_speed = True

            if self.home_reached_cruise_speed and (
                abs(self.velocity_rps)
                <= target_home_speed * self.home_velocity_cutoff_ratio
            ):
                self.home_velocity_cutoff_count += 1
            else:
                self.home_velocity_cutoff_count = 0

            velocity_cutoff_reached = (
                self.home_velocity_cutoff_count >= self.home_velocity_cutoff_samples
            )

        if current_cutoff_reached or velocity_cutoff_reached:
            self.home_backoff_active = True
            self.home_backoff_target_rev = (
                self.position_rev + self.home_backoff_direction() * self.home_backoff_rotations
            )
            if velocity_cutoff_reached:
                self.get_logger().info(
                    "Lin Act Home cutoff by velocity drop "
                    f"({self.velocity_rps:.2f} rps <= "
                    f"{abs(self.home_velocity_rps) * self.home_velocity_cutoff_ratio:.2f} rps)"
                )
            else:
                self.get_logger().info(
                    "Lin Act Home cutoff by current threshold "
                    f"({self.iq_measured_amps:.2f} A)"
                )
            self.send_velocity(self.home_backoff_velocity_command())
            return

        self.send_velocity(self.clamp_velocity(self.home_velocity_rps))

    def home_backoff_direction(self):
        return 1.0 if self.home_velocity_rps < 0.0 else -1.0

    def home_backoff_velocity_command(self):
        return self.clamp_velocity(
            self.home_backoff_direction() * self.home_backoff_velocity_rps
        )

    def has_reached_home_backoff_target(self):
        if self.home_backoff_direction() > 0.0:
            return self.position_rev >= self.home_backoff_target_rev
        return self.position_rev <= self.home_backoff_target_rev

    def is_past_home_rampup(self):
        if self.input_vel_ramp <= 0.0:
            return True
        rampup_time_s = 2* abs(self.home_velocity_rps) / self.input_vel_ramp
        return time() >= self.home_start_time + rampup_time_s + self.home_rampup_margin_s

    def reset_failsafe_callback(self, request, response):
        if not request.data:
            response.success = True
            response.message = "Failsafe state unchanged"
            return response

        self.failsafe_active = False
        self.command_velocity_rps = 0.0
        self.last_command_time = 0.0

        try:
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_CLEAR_ERRORS),
                    data=struct.pack("<I", 1),
                    is_extended_id=False,
                )
            )
            self.send_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
            response.success = True
            response.message = "Linear actuator failsafe reset"
        except Exception as exc:
            response.success = False
            response.message = f"Failed to reset linear actuator failsafe: {exc}"

        return response

    def check_current_failsafe(self):
        if self.current_limit_amps <= 0.0 or self.failsafe_active:
            return

        if abs(self.iq_measured_amps) <= self.current_limit_amps:
            return

        self.failsafe_active = True
        self.command_velocity_rps = 0.0
        self.send_velocity(0.0)
        if self.idle_on_failsafe:
            self.send_axis_state(AXIS_STATE_IDLE)
        self.get_logger().error(
            "Linear actuator current limit exceeded: "
            f"{self.iq_measured_amps:.2f} A > {self.current_limit_amps:.2f} A"
        )

    def read_can(self):
        for can_msg in self.get_can_buffer():
            node_id = (can_msg.arbitration_id >> 5) & ((1 << 6) - 1)
            if node_id != self.node_id:
                continue

            cmd_id = can_msg.arbitration_id & ((1 << 5) - 1)
            try:
                if cmd_id == CMD_GET_ENCODER_ESTIMATES:
                    self.position_rev, self.velocity_rps = struct.unpack(
                        "<ff",
                        bytes(can_msg.data),
                    )
                elif cmd_id == CMD_GET_IQ:
                    self.iq_set_amps, self.iq_measured_amps = struct.unpack(
                        "<ff",
                        bytes(can_msg.data),
                    )
            except struct.error as exc:
                self.get_logger().warn(f"Malformed linear actuator CAN frame: {exc}")

    def request_telemetry(self):
        if time() < self.last_telemetry_request_time + self.telemetry_request_period_s:
            return

        self.last_telemetry_request_time = time()
        try:
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_GET_ENCODER_ESTIMATES),
                    is_extended_id=False,
                    is_remote_frame=True,
                )
            )
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_GET_IQ),
                    is_extended_id=False,
                    is_remote_frame=True,
                )
            )
        except Exception as exc:
            self.get_logger().warn(f"Linear actuator telemetry request error: {exc}")

    def publish_telemetry(self):
        """
        This could potentially be unneeded. We can scrap the publishers if we don't need to access the telemetry
        data elseswhere, though Unity tie-ins may appreciate this.
        """
        position_msg = Float32()
        position_msg.data = float(self.get_relative_position_rotations())
        self.position_pub.publish(position_msg)

        current_msg = Float32()
        current_msg.data = float(self.iq_measured_amps)
        self.current_pub.publish(current_msg)

        failsafe_msg = Bool()
        failsafe_msg.data = self.failsafe_active
        self.failsafe_pub.publish(failsafe_msg)

    def send_velocity(self, velocity_rps):
        try:
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_SET_INPUT_VEL),
                    data=struct.pack("<ff", float(velocity_rps), 0.0),
                    is_extended_id=False,
                )
            )
        except Exception as exc:
            self.get_logger().warn(f"Linear actuator velocity command error: {exc}")

    def send_axis_state(self, axis_state):
        self.bus.send(
            can.Message(
                arbitration_id=self.arbitration_id(CMD_SET_AXIS_STATE),
                data=struct.pack("<I", int(axis_state)),
                is_extended_id=False,
            )
        )

    def flush_can_buffer(self):
        while self.bus.recv(timeout=0) is not None:
            pass

    def get_can_buffer(self):
        can_msgs = []
        for _ in range(1000):
            can_msg = self.bus.recv(timeout=0)
            if can_msg is None:
                break
            can_msgs.append(can_msg)
        return can_msgs

    def arbitration_id(self, cmd_id):
        return self.node_id << 5 | cmd_id

    def clamp_velocity(self, velocity_rps):
        """
        Returns 0 if not a number, otherwise returns the maximum of the negative of the velocity
        limit; and the velocity limit or the given velocity requested, to prevent going too fast.
        """
        if not math.isfinite(velocity_rps):
            return 0.0
        return max(-self.velocity_limit_rps, min(self.velocity_limit_rps, velocity_rps))

    def get_relative_position_rotations(self):
        return self.position_rev - self.home_zero_position_rev

    def limit_velocity_by_position(self, requested_velocity_rps):
        limited_velocity = self.clamp_velocity(requested_velocity_rps)
        rel_pos = self.get_relative_position_rotations()
        accel_limit = max(self.input_vel_ramp, 1e-6)

        dist_to_min = rel_pos - self.min_position_rotations
        dist_to_max = self.max_position_rotations - rel_pos

        if dist_to_min <= 0.0:
            return max(0.0, limited_velocity)
        if dist_to_max <= 0.0:
            return min(0.0, limited_velocity)

        max_negative_speed = math.sqrt(2.0 * accel_limit * dist_to_min)
        max_positive_speed = math.sqrt(2.0 * accel_limit * dist_to_max)

        if limited_velocity > 0.0:
            return min(limited_velocity, max_positive_speed)
        if limited_velocity < 0.0:
            return max(limited_velocity, -max_negative_speed)
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = LinearActuatorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
