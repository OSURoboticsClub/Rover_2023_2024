#!/usr/bin/python3
"""Exercise the science pump/valve interlock with edge-case heartbeats."""

from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rover2_spectrometry_interface.msg import (
    SpectrometryMechanicalControlMessage,
)
from rover2_spectrometry_interface.msg import (
    SpectrometryMechanicalStatusMessage,
)


class MechanicalInterlockTest(Node):
    """Publish pump/valve edge cases and check the reported applied state."""

    SCENARIOS = (
        "delayed_pump",
        "alternating_invalid",
        "rotating_valves",
    )

    def __init__(self):
        super().__init__("mechanical_interlock_test")

        self.declare_parameter("scenario", "all")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("scenario_duration_s", 3.0)
        self.declare_parameter("command_topic", "science/mechanical/control")
        self.declare_parameter("status_topic", "science/mechanical/status")

        scenario = str(self.get_parameter("scenario").value)
        if scenario == "all":
            self.scenarios = self.SCENARIOS
        elif scenario in self.SCENARIOS:
            self.scenarios = (scenario,)
        else:
            choices = ", ".join(("all",) + self.SCENARIOS)
            raise ValueError(f"scenario must be one of: {choices}")

        publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )
        self.scenario_duration_s = max(
            0.6, float(self.get_parameter("scenario_duration_s").value)
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            SpectrometryMechanicalControlMessage,
            self.get_parameter("command_topic").value,
            qos,
        )
        self.status_subscription = self.create_subscription(
            SpectrometryMechanicalStatusMessage,
            self.get_parameter("status_topic").value,
            self.status_callback,
            qos,
        )

        self.sequence = 1
        self.publish_count = 0
        self.scenario_index = 0
        self.scenario_started_at = monotonic()
        self.expected_states = {}
        self.status_checks = 0
        self.status_failures = 0
        self.finished_at = None
        self.timer = self.create_timer(
            1.0 / publish_rate_hz, self.timer_callback
        )
        self._log_scenario_start()

    def timer_callback(self):
        """Publish the current scenario at the configured heartbeat rate."""
        now = monotonic()
        if self.finished_at is not None:
            self._publish_state(False, False, False, False)
            if now >= self.finished_at + 0.5:
                self.get_logger().info(
                    "Interlock test complete: "
                    f"{self.status_checks} statuses checked, "
                    f"{self.status_failures} failures"
                )
                rclpy.shutdown()
            return

        elapsed = now - self.scenario_started_at
        if elapsed >= self.scenario_duration_s:
            self.scenario_index += 1
            if self.scenario_index >= len(self.scenarios):
                self.finished_at = now
                self._publish_state(False, False, False, False)
                return
            self.scenario_started_at = now
            self.publish_count = 0
            elapsed = 0.0
            self._log_scenario_start()

        valve_1_on, valve_2_on, pump_on = self._state_for_scenario(
            self.scenarios[self.scenario_index], elapsed
        )
        self._publish_state(True, valve_1_on, valve_2_on, pump_on)
        self.publish_count += 1

    def _state_for_scenario(self, scenario, elapsed):
        """Return valve 1, valve 2, and pump requests for one heartbeat."""
        if scenario == "delayed_pump":
            return True, False, elapsed >= 0.5

        if scenario == "alternating_invalid":
            if self.publish_count % 2 == 0:
                return False, False, True
            return True, True, False

        # Alternate invalid pump-only requests with valve 1, valve 2, and both.
        if self.publish_count % 2 == 0:
            return False, False, True
        valve_states = ((True, False), (False, True), (True, True))
        valve_index = (self.publish_count // 2) % len(valve_states)
        valve_1_on, valve_2_on = valve_states[valve_index]
        return valve_1_on, valve_2_on, True

    def _publish_state(
        self, controls_unlocked, valve_1_on, valve_2_on, pump_on
    ):
        """Publish one complete heartbeat and retain its expected result."""
        msg = SpectrometryMechanicalControlMessage()
        msg.sequence = self.sequence
        msg.controls_unlocked = controls_unlocked
        msg.valve_1_on = valve_1_on
        msg.valve_2_on = valve_2_on
        msg.pump_on = pump_on
        msg.coil_1_on = False
        msg.coil_2_on = False
        self.publisher.publish(msg)

        effective_pump_on = (
            controls_unlocked
            and pump_on
            and (valve_1_on or valve_2_on)
        )
        self.expected_states[self.sequence] = (
            controls_unlocked,
            valve_1_on,
            valve_2_on,
            effective_pump_on,
        )
        if len(self.expected_states) > 500:
            self.expected_states.pop(next(iter(self.expected_states)))
        self.sequence = (self.sequence + 1) % (2**32)

    def status_callback(self, msg):
        """Check that reported state matches the interlocked request."""
        if not msg.command_state_valid or not msg.command_link_active:
            return
        expected = self.expected_states.get(int(msg.command_sequence))
        if expected is None:
            return

        self.status_checks += 1
        actual = (
            bool(msg.controls_unlocked),
            bool(msg.valve_1_on),
            bool(msg.valve_2_on),
            bool(msg.pump_on),
        )
        unsafe = actual[3] and not (actual[1] or actual[2])
        if actual != expected or unsafe:
            self.status_failures += 1
            self.get_logger().error(
                f"Sequence {msg.command_sequence}: expected {expected}, "
                f"received {actual}"
            )

    def _log_scenario_start(self):
        scenario = self.scenarios[self.scenario_index]
        self.get_logger().info(f"Starting interlock scenario: {scenario}")


def main(args=None):
    """Run the mechanical interlock edge-case publisher."""
    rclpy.init(args=args)
    node = MechanicalInterlockTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
