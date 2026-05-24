#!/usr/bin/python3

from time import monotonic
import math
import struct

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_TORQUE = 0x0E
CMD_GET_IQ = 0x14

CONTROL_MODE_TORQUE_CONTROL = 1
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_TORQUE_RAMP = 6


class DrillControl(Node):
    def __init__(self):
        super().__init__("drill_control")

        self.declare_parameter("can", "can0")
        self.declare_parameter("node_id", 2)
        self.declare_parameter("command_topic", "drill/control")
        self.declare_parameter("speed_topic", "drill/speed")
        self.declare_parameter("current_topic", "drill/current")
        self.declare_parameter("timer_period_s", 0.02)
        self.declare_parameter("telemetry_request_period_s", 0.05)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("command_qos_depth", 1)
        self.declare_parameter("can_send_timeout_s", 0.005)
        self.declare_parameter("max_can_frames_per_cycle", 50)
        self.declare_parameter("use_torque_ramp_input_mode", True)
        self.declare_parameter("idle_on_disable_or_timeout", True)
        self.declare_parameter("idle_only_on_timeout", True)
        self.declare_parameter("direction", -1.0)
        self.declare_parameter("full_speed_torque_nm", 1)
        self.declare_parameter("command_on_threshold", 0.5)
        self.declare_parameter("torque_ramp_rate_nmps", .8)
        self.declare_parameter("velocity_limit_rps", 25.0)
        self.declare_parameter("odrive_vel_limit_endpoint", 401)
        self.declare_parameter("odrive_torque_ramp_rate_endpoint", 404)

        self.can_bus = self.get_parameter("can").value
        self.node_id = int(self.get_parameter("node_id").value)
        self.telemetry_request_period_s = float(
            self.get_parameter("telemetry_request_period_s").value
        )
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.command_qos_depth = max(
            1, int(self.get_parameter("command_qos_depth").value)
        )
        self.can_send_timeout_s = max(
            0.0, float(self.get_parameter("can_send_timeout_s").value)
        )
        self.max_can_frames_per_cycle = max(
            1, int(self.get_parameter("max_can_frames_per_cycle").value)
        )
        self.use_torque_ramp_input_mode = bool(
            self.get_parameter("use_torque_ramp_input_mode").value
        )
        self.idle_on_disable_or_timeout = bool(
            self.get_parameter("idle_on_disable_or_timeout").value
        )
        self.idle_only_on_timeout = bool(
            self.get_parameter("idle_only_on_timeout").value
        )
        self.direction = 1.0 if float(self.get_parameter("direction").value) >= 0.0 else -1.0
        self.full_speed_torque_nm = abs(
            float(self.get_parameter("full_speed_torque_nm").value)
        )
        self.command_on_threshold = float(self.get_parameter("command_on_threshold").value)
        self.torque_ramp_rate_nmps = max(
            1e-6, abs(float(self.get_parameter("torque_ramp_rate_nmps").value))
        )
        self.velocity_limit_rps = abs(
            float(self.get_parameter("velocity_limit_rps").value)
        )
        self.odrive_vel_limit_endpoint = int(
            self.get_parameter("odrive_vel_limit_endpoint").value
        )
        self.odrive_torque_ramp_rate_endpoint = int(
            self.get_parameter("odrive_torque_ramp_rate_endpoint").value
        )

        self.command_enabled = False
        self.command_torque_nm = 0.0
        self.velocity_rps = 0.0
        self.iq_set_amps = 0.0
        self.iq_measured_amps = 0.0
        self.last_command_time = monotonic() - self.command_timeout_s - 1.0
        self.last_telemetry_request_time = monotonic()
        self.in_closed_loop = False

        self.bus = can.interface.Bus(self.can_bus, interface="socketcan")
        self.flush_can_buffer()
        self.setup_controller()

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.command_qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.command_subscription = self.create_subscription(
            Float32,
            self.get_parameter("command_topic").value,
            self.command_callback,
            command_qos,
        )
        self.speed_pub = self.create_publisher(
            Float32,
            self.get_parameter("speed_topic").value,
            10,
        )
        self.current_pub = self.create_publisher(
            Float32,
            self.get_parameter("current_topic").value,
            10,
        )

        self.timer = self.create_timer(
            float(self.get_parameter("timer_period_s").value), self.timer_callback
        )

    def setup_controller(self):
        try:
            input_mode = (
                INPUT_MODE_TORQUE_RAMP
                if self.use_torque_ramp_input_mode
                else INPUT_MODE_PASSTHROUGH
            )
            self.enter_closed_loop()
            self.send_can_message(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_SET_CONTROLLER_MODE),
                    data=struct.pack("<II", CONTROL_MODE_TORQUE_CONTROL, input_mode),
                    is_extended_id=False,
                )
            )
            self.send_can_message(
                can.Message(
                    arbitration_id=self.arbitration_id(0x04),
                    data=struct.pack(
                        "<BHBf",
                        1,
                        self.odrive_vel_limit_endpoint,
                        0,
                        self.velocity_limit_rps,
                    ),
                    is_extended_id=False,
                )
            )
            if self.use_torque_ramp_input_mode:
                self.send_can_message(
                    can.Message(
                        arbitration_id=self.arbitration_id(0x04),
                        data=struct.pack(
                            "<BHBf",
                            1,
                            self.odrive_torque_ramp_rate_endpoint,
                            0,
                            self.torque_ramp_rate_nmps,
                        ),
                        is_extended_id=False,
                    )
                )
        except Exception as exc:
            self.get_logger().error(f"Drill CAN setup error: {exc}")

    def command_callback(self, msg):
        self.command_enabled = float(msg.data) >= self.command_on_threshold
        if self.command_enabled:
            self.enter_closed_loop()
        self.command_torque_nm = (
            self.direction * self.full_speed_torque_nm if self.command_enabled else 0.0
        )
        if (
            not self.command_enabled
            and self.idle_on_disable_or_timeout
            and not self.idle_only_on_timeout
        ):
            self.send_torque(0.0)
            self.enter_idle()
        self.last_command_time = monotonic()

    def timer_callback(self):
        now = monotonic()

        if now >= self.last_command_time + self.command_timeout_s:
            self.command_enabled = False
            self.command_torque_nm = 0.0
            if self.idle_on_disable_or_timeout:
                self.send_torque(0.0)
                self.enter_idle()
        elif self.command_enabled:
            self.enter_closed_loop()

        self.send_torque(self.command_torque_nm)
        self.read_can()
        self.request_telemetry(now)

        self.publish_telemetry()

    def read_can(self):
        for can_msg in self.get_can_buffer():
            node_id = (can_msg.arbitration_id >> 5) & ((1 << 6) - 1)
            if node_id != self.node_id:
                continue

            cmd_id = can_msg.arbitration_id & ((1 << 5) - 1)
            try:
                if cmd_id == CMD_GET_ENCODER_ESTIMATES:
                    _, self.velocity_rps = struct.unpack("<ff", bytes(can_msg.data))
                elif cmd_id == CMD_GET_IQ:
                    self.iq_set_amps, self.iq_measured_amps = struct.unpack(
                        "<ff", bytes(can_msg.data)
                    )
            except struct.error as exc:
                self.get_logger().warn(f"Malformed drill CAN frame: {exc}")

    def request_telemetry(self, now):
        if now < self.last_telemetry_request_time + self.telemetry_request_period_s:
            return

        self.last_telemetry_request_time = now
        try:
            self.send_can_message(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_GET_ENCODER_ESTIMATES),
                    is_extended_id=False,
                    is_remote_frame=True,
                )
            )
            self.send_can_message(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_GET_IQ),
                    is_extended_id=False,
                    is_remote_frame=True,
                )
            )
        except Exception as exc:
            self.get_logger().warn(f"Drill telemetry request error: {exc}")

    def publish_telemetry(self):
        speed_msg = Float32()
        speed_msg.data = float(self.velocity_rps)
        self.speed_pub.publish(speed_msg)

        current_msg = Float32()
        current_msg.data = float(self.iq_measured_amps)
        self.current_pub.publish(current_msg)

    def send_torque(self, torque_nm):
        if not self.in_closed_loop:
            return
        safe_torque = 0.0 if not math.isfinite(torque_nm) else float(torque_nm)
        try:
            self.send_can_message(
                can.Message(
                    arbitration_id=self.arbitration_id(CMD_SET_INPUT_TORQUE),
                    data=struct.pack("<f", safe_torque),
                    is_extended_id=False,
                )
            )
        except Exception as exc:
            self.get_logger().warn(f"Drill torque command error: {exc}")

    def send_axis_state(self, axis_state):
        self.send_can_message(
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
        for _ in range(self.max_can_frames_per_cycle):
            can_msg = self.bus.recv(timeout=0)
            if can_msg is None:
                break
            can_msgs.append(can_msg)
        return can_msgs

    def send_can_message(self, msg):
        self.bus.send(msg, timeout=self.can_send_timeout_s)

    def enter_closed_loop(self):
        if self.in_closed_loop:
            return
        self.send_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.in_closed_loop = True

    def enter_idle(self):
        if not self.in_closed_loop:
            return
        self.send_axis_state(AXIS_STATE_IDLE)
        self.in_closed_loop = False

    def arbitration_id(self, cmd_id):
        return self.node_id << 5 | cmd_id


def main(args=None):
    rclpy.init(args=args)
    node = DrillControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
