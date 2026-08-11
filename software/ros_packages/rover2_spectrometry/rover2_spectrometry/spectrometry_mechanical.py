#!/usr/bin/python3
"""Bridge groundstation science-mechanism heartbeats to rover CAN."""

from time import monotonic

import can
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rover2_spectrometry_interface.msg import (
    SpectrometryMechanicalControlMessage,
)
from rover2_spectrometry_interface.msg import (
    SpectrometryMechanicalStatusMessage,
)

from .protocol import (
    MechanicalState,
    WATCHDOG_TIMEOUT_STATE,
    command_codes_for_transition,
)


class SpectrometryMechanical(Node):
    """Apply mechanical state commands with a ROS-link watchdog."""

    def __init__(self):
        super().__init__("spectrometry_mechanical")

        self.declare_parameter("can_bus", "can0")
        self.declare_parameter("node_id", 60)
        self.declare_parameter("command_topic", "science/mechanical/control")
        self.declare_parameter("status_topic", "science/mechanical/status")
        self.declare_parameter("timer_period_s", 0.02)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("command_qos_depth", 1)
        self.declare_parameter("can_send_timeout_s", 0.005)

        self.can_interface = str(self.get_parameter("can_bus").value)
        self.node_id = int(self.get_parameter("node_id").value)
        self.command_timeout_s = max(
            0.05, float(self.get_parameter("command_timeout_s").value)
        )
        self.can_send_timeout_s = max(
            0.0, float(self.get_parameter("can_send_timeout_s").value)
        )
        if not 0 <= self.node_id <= 63:
            raise ValueError(
                "node_id must fit the six-bit standard CAN layout"
            )

        self.requested_state = WATCHDOG_TIMEOUT_STATE
        self.requested_sequence = 0
        self.applied_state = None
        self.applied_sequence = 0
        self.last_command_time = monotonic() - self.command_timeout_s - 1.0
        self.command_link_was_active = False

        self.bus = can.interface.Bus(
            channel=self.can_interface,
            interface="socketcan",
        )
        self.get_logger().info(
            f"Science CAN opened on {self.can_interface}, node {self.node_id}"
        )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(self.get_parameter("command_qos_depth").value)),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.command_subscription = self.create_subscription(
            SpectrometryMechanicalControlMessage,
            self.get_parameter("command_topic").value,
            self.command_callback,
            command_qos,
        )
        self.status_publisher = self.create_publisher(
            SpectrometryMechanicalStatusMessage,
            self.get_parameter("status_topic").value,
            command_qos,
        )
        self.timer = self.create_timer(
            max(0.005, float(self.get_parameter("timer_period_s").value)),
            self.timer_callback,
        )

    def command_callback(self, msg):
        """Latch a complete state heartbeat and refresh its lease."""
        now = monotonic()
        heartbeat_gap_expired = (
            now >= self.last_command_time + self.command_timeout_s
        )
        if heartbeat_gap_expired:
            self.requested_state = WATCHDOG_TIMEOUT_STATE
            self.command_link_was_active = False

        state = MechanicalState(
            controls_unlocked=bool(msg.controls_unlocked),
            valve_1_on=bool(msg.valve_1_on),
            valve_2_on=bool(msg.valve_2_on),
            pump_on=bool(msg.pump_on),
            coil_1_on=bool(msg.coil_1_on),
            coil_2_on=bool(msg.coil_2_on),
        ).normalized()
        self.last_command_time = now

        self.requested_state = state
        self.requested_sequence = int(msg.sequence)

    def timer_callback(self):
        """Apply transitions, enforce timeout safety, and publish status."""
        now = monotonic()
        command_link_active = (
            now < self.last_command_time + self.command_timeout_s
        )

        if self.command_link_was_active and not command_link_active:
            self.get_logger().warning(
                "Science command heartbeat timed out; turning all outputs off"
            )
            self.requested_state = WATCHDOG_TIMEOUT_STATE

        self.command_link_was_active = command_link_active
        target_state = (
            self.requested_state
            if command_link_active
            else WATCHDOG_TIMEOUT_STATE
        )

        if target_state != self.applied_state:
            self._transmit_transition(self.applied_state, target_state)
            self.applied_state = target_state
        if command_link_active:
            self.applied_sequence = self.requested_sequence

        self._publish_status(command_link_active)

    def _transmit_transition(self, previous_state, target_state):
        """Send only the CAN commands needed for a state transition."""
        command_codes = command_codes_for_transition(
            previous_state, target_state
        )
        if not command_codes:
            return

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        for command_code in command_codes:
            # Commented out code below tests the functionality of the CAN messages.
            # message = can.Message(
            #     arbitration_id=self.arbitration_id(command_code),
            #     data=[],
            #     is_extended_id=False,
            #     is_rx=False,
            # )
            # print(
            #     f"\n{command_code.name} "
            #     f"(command={int(command_code)}, "
            #     f"arbitration_id=0x{message.arbitration_id:03X}): "
            #     f"{message}"
            # )
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(command_code),
                    data=[],
                    is_extended_id=False,
                ),
                timeout=self.can_send_timeout_s,
            )

    def _publish_status(self, command_link_active):
        state = self.applied_state or WATCHDOG_TIMEOUT_STATE
        msg = SpectrometryMechanicalStatusMessage()
        msg.command_sequence = self.applied_sequence
        msg.command_link_active = command_link_active
        msg.can_connected = True
        msg.command_state_valid = self.applied_state is not None
        msg.controls_unlocked = state.controls_unlocked
        msg.valve_1_on = state.valve_1_on
        msg.valve_2_on = state.valve_2_on
        msg.pump_on = state.pump_on
        msg.coil_1_on = state.coil_1_on
        msg.coil_2_on = state.coil_2_on
        self.status_publisher.publish(msg)

    def arbitration_id(self, command_code):
        """Combine the six-bit node and five-bit command IDs."""
        return (self.node_id << 5) | int(command_code)

    def destroy_node(self):
        """Best-effort safe shutdown before releasing the CAN interface."""
        self._transmit_transition(
            self.applied_state, WATCHDOG_TIMEOUT_STATE
        )
        self.bus.shutdown()
        return super().destroy_node()


def main(args=None):
    """Run the rover-side science mechanical bridge."""
    rclpy.init(args=args)
    node = SpectrometryMechanical()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
