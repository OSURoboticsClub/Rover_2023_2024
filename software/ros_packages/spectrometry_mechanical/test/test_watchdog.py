"""Behavior tests for the rover-side heartbeat watchdog."""

from time import monotonic

import pytest
import rclpy

try:
    from rover2_spectrometry_interface.msg import (
        SpectrometryMechanicalControlMessage,
    )
    import spectrometry_mechanical.spectrometery_mechanical as bridge_module
except (ImportError, ModuleNotFoundError):
    pytest.skip(
        "generated rover2_spectrometry_interface messages are not built",
        allow_module_level=True,
    )


class FakeBus:
    """Collect CAN messages without requiring a SocketCAN interface."""

    def __init__(self):
        self.messages = []
        self.was_shutdown = False

    def send(self, message, timeout):
        self.messages.append((message, timeout))

    def shutdown(self):
        self.was_shutdown = True


def command_ids(fake_bus):
    """Return only the five-bit command portion of captured CAN IDs."""
    return [message.arbitration_id & 0x1F for message, _ in fake_bus.messages]


def test_timeout_turns_outputs_off_and_resumes_immediately(monkeypatch):
    fake_bus = FakeBus()
    monkeypatch.setattr(
        bridge_module.can.interface,
        "Bus",
        lambda **_kwargs: fake_bus,
    )

    rclpy.init()
    node = bridge_module.SpectrometryMechanical()
    try:
        assert node.arbitration_id(7) == 0x787
        assert node.arbitration_id(24) == 0x798

        node.timer_callback()
        assert command_ids(fake_bus) == [8, 14, 16, 22, 24]

        locked = SpectrometryMechanicalControlMessage()
        locked.sequence = 1
        node.command_callback(locked)
        node.timer_callback()
        active = SpectrometryMechanicalControlMessage()
        active.sequence = 2
        active.controls_unlocked = True
        active.pump_on = True
        node.command_callback(active)
        node.timer_callback()
        assert command_ids(fake_bus)[-5:] == [8, 14, 15, 22, 24]

        repeated = SpectrometryMechanicalControlMessage()
        repeated.sequence = 3
        repeated.controls_unlocked = True
        repeated.pump_on = True
        message_count = len(fake_bus.messages)
        node.command_callback(repeated)
        node.timer_callback()
        assert len(fake_bus.messages) == message_count + 5

        resumed_after_gap = SpectrometryMechanicalControlMessage()
        resumed_after_gap.sequence = 4
        resumed_after_gap.controls_unlocked = True
        resumed_after_gap.coil_1_on = True
        node.last_command_time = monotonic() - node.command_timeout_s - 0.1
        node.command_callback(resumed_after_gap)
        node.timer_callback()
        assert command_ids(fake_bus)[-6:] == [8, 14, 16, 17, 20, 24]
        assert node.applied_state.coil_1_on

        message_count = len(fake_bus.messages)
        node.timer_callback()
        assert len(fake_bus.messages) == message_count

        node.last_command_time = monotonic() - node.command_timeout_s - 0.1
        node.timer_callback()
        assert node.applied_state == bridge_module.WATCHDOG_TIMEOUT_STATE
        assert command_ids(fake_bus)[-5:] == [8, 14, 16, 22, 24]

        active.sequence = 5
        node.command_callback(active)
        node.timer_callback()
        assert node.applied_state.pump_on

    finally:
        node.destroy_node()
        rclpy.shutdown()
