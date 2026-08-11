#!/usr/bin/python3
"""CLI helper for publishing linear actuator velocity commands."""

import argparse
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LinearActuatorCommandNode(Node):
    """Minimal ROS publisher node for manual command testing."""

    def __init__(self, topic_name):
        super().__init__("linear_actuator_command")
        self.publisher = self.create_publisher(Float32, topic_name, 10)

    def publish_velocity(self, velocity_rps):
        msg = Float32()
        msg.data = float(velocity_rps)
        self.publisher.publish(msg)


def parse_args():
    """Parse CLI arguments for topic, value, and burst settings."""
    parser = argparse.ArgumentParser(
        description="Publish velocity commands for linear actuator control"
    )
    parser.add_argument("velocity", type=float, help="Command velocity in rev/s")
    parser.add_argument(
        "--topic",
        default="linear_actuator/control",
        help="Topic name for commands",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of messages to publish",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Publish rate in hz when count > 1",
    )
    return parser.parse_args()


def main(args=None):
    """Publish one or more command messages and exit."""
    cli = parse_args()
    rclpy.init(args=args)
    node = LinearActuatorCommandNode(cli.topic)

    publish_count = max(1, int(cli.count))
    publish_rate = max(0.1, float(cli.rate))
    period_s = 1.0 / publish_rate

    for _ in range(publish_count):
        node.publish_velocity(cli.velocity)
        rclpy.spin_once(node, timeout_sec=0.01)
        if publish_count > 1:
            sleep(period_s)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
