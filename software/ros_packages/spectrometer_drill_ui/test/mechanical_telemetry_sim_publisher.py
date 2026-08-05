#!/usr/bin/env python3
"""Synthetic telemetry publisher for UI chart testing without hardware."""

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MechanicalTelemetrySimPublisher(Node):
    """Publish increasing, noisy telemetry traces on all four chart topics."""

    def __init__(self):
        super().__init__("mechanical_telemetry_sim_publisher")

        self.drill_speed_pub = self.create_publisher(Float32, "drill/speed", 10)
        self.drill_current_pub = self.create_publisher(Float32, "drill/current", 10)
        self.linear_position_pub = self.create_publisher(
            Float32, "linear_actuator/position", 10
        )
        self.linear_current_pub = self.create_publisher(
            Float32, "linear_actuator/current", 10
        )

        self.time_s = 0.0
        self.timer_period_s = 0.1
        self.timer = self.create_timer(self.timer_period_s, self._timer_callback)

    def _timer_callback(self):
        """Advance baseline signals each tick and publish noisy samples."""
        self.time_s += self.timer_period_s

        drill_speed_base = 0.35 * self.time_s
        drill_current_base = 1.20 + (0.10 * self.time_s)
        linear_position_base = 8.0 * self.time_s
        linear_current_base = 0.90 + (0.08 * self.time_s)

        self._publish_with_noise(self.drill_speed_pub, drill_speed_base)
        self._publish_with_noise(self.drill_current_pub, drill_current_base)
        self._publish_with_noise(self.linear_position_pub, linear_position_base)
        self._publish_with_noise(self.linear_current_pub, linear_current_base)

    def _publish_with_noise(self, publisher, base_value):
        """Publish one sample with bounded random noise (+/-20% of baseline)."""
        noise_limit = abs(base_value) * 0.20
        noisy_value = base_value + random.uniform(-noise_limit, noise_limit)
        msg = Float32()
        msg.data = float(max(0.0, noisy_value))
        publisher.publish(msg)


def main(args=None):
    """Run until interrupted."""
    rclpy.init(args=args)
    node = MechanicalTelemetrySimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
