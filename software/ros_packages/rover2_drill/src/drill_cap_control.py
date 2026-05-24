#!/usr/bin/python3
"""Drill cap CAN service node.

Provides a ROS service that maps engaged/disengaged cap states to a single
byte CAN command for the drill cap actuator.
"""

import struct

import can
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class DrillCapControl(Node):
    """Bridge `SetBool` cap commands to CAN payloads."""

    def __init__(self):
        super().__init__("drill_cap_control")

        self.declare_parameter("can", "can0")
        self.declare_parameter("node_id", 0)
        self.declare_parameter("cmd_id", 1)
        self.declare_parameter("service_name", "drill/cap/set_engaged")
        self.declare_parameter("can_send_timeout_s", 0.01)
        self.declare_parameter("send_on_change_only", False)

        self.can_bus = self.get_parameter("can").value
        self.node_id = int(self.get_parameter("node_id").value)
        self.cmd_id = int(self.get_parameter("cmd_id").value)
        self.can_send_timeout_s = max(
            0.0, float(self.get_parameter("can_send_timeout_s").value)
        )
        self.send_on_change_only = bool(
            self.get_parameter("send_on_change_only").value
        )

        # Used when `send_on_change_only` is enabled.
        self.last_state = None

        # Single CAN bus connection for cap state messages.
        self.bus = can.interface.Bus(self.can_bus, interface="socketcan")

        self.service = self.create_service(
            SetBool,
            self.get_parameter("service_name").value,
            self.set_cap_state_callback,
        )

        # Default safe state at startup is disengaged.
        self.send_cap_state(False)

    def send_cap_state(self, desired_state):
        """Send cap state to the actuator; optionally suppress duplicate writes."""
        if self.send_on_change_only and desired_state == self.last_state:
            return True

        self.last_state = desired_state
        payload = struct.pack("<Bxxxxxxx", 0 if desired_state else 1)

        try:
            self.bus.send(
                can.Message(
                    arbitration_id=self.arbitration_id(),
                    data=payload,
                    is_extended_id=False,
                ),
                timeout=self.can_send_timeout_s,
            )
            return True
        except Exception as exc:
            self.get_logger().warn(f"Drill cap CAN send error: {exc}")
            return False

    def set_cap_state_callback(self, request, response):
        """Service callback wrapper around `send_cap_state`."""
        desired_state = bool(request.data)
        response.success = self.send_cap_state(desired_state)
        if response.success:
            response.message = "engaged" if desired_state else "disengaged"
        else:
            response.message = "failed to send CAN message"
        return response

    def arbitration_id(self):
        return (self.node_id << 5) | self.cmd_id


def main(args=None):
    rclpy.init(args=args)
    node = DrillCapControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
