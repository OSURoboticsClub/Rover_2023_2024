"""Tests for science mechanism state-to-CAN transitions."""

from rover2_spectrometry.protocol import (
    CommandCode,
    MechanicalState,
    WATCHDOG_TIMEOUT_STATE,
    command_codes_for_transition,
)


def test_startup_sends_complete_all_off_state():
    """An unknown startup state must explicitly converge every output off."""
    assert command_codes_for_transition(None, WATCHDOG_TIMEOUT_STATE) == (
        CommandCode.ALL_VALVES_OFF,
        CommandCode.PUMP_OFF,
        CommandCode.ALL_COILS_OFF,
        CommandCode.SOLENOID_OFF,
    )


def test_startup_locked_state_explicitly_turns_every_device_off():
    """Locking must explicitly turn every physical output off."""
    assert command_codes_for_transition(None, MechanicalState()) == (
        CommandCode.VALVE_1_OFF,
        CommandCode.VALVE_2_OFF,
        CommandCode.PUMP_OFF,
        CommandCode.COIL_1_OFF,
        CommandCode.COIL_2_OFF,
        CommandCode.SOLENOID_OFF,
    )


def test_unchanged_state_sends_nothing():
    """A heartbeat containing the applied state must not retrigger devices."""
    state = MechanicalState(
        controls_unlocked=True,
        pump_on=True,
        coil_1_on=True,
    )
    assert command_codes_for_transition(state, state) == ()


def test_only_changed_device_is_sent():
    """Changing the pump must not resend an already-running coil command."""
    previous = MechanicalState(
        controls_unlocked=True,
        coil_1_on=True,
    )
    target = MechanicalState(
        controls_unlocked=True,
        pump_on=True,
        coil_1_on=True,
    )
    assert command_codes_for_transition(previous, target) == (
        CommandCode.PUMP_ON,
    )


def test_lock_explicitly_turns_off_active_devices():
    """Locking must explicitly turn active physical outputs off."""
    previous = MechanicalState(
        controls_unlocked=True,
        valve_1_on=True,
        valve_2_on=True,
        pump_on=True,
        coil_1_on=True,
        coil_2_on=True,
        solenoid_on=True,
    )
    target = MechanicalState(controls_unlocked=False)
    assert command_codes_for_transition(previous, target) == (
        CommandCode.VALVE_1_OFF,
        CommandCode.VALVE_2_OFF,
        CommandCode.PUMP_OFF,
        CommandCode.COIL_1_OFF,
        CommandCode.COIL_2_OFF,
        CommandCode.SOLENOID_OFF,
    )


def test_unlock_only_turns_on_requested_devices():
    """Unlocking sends only commands for requested device changes."""
    previous = MechanicalState(controls_unlocked=False)
    target = MechanicalState(
        controls_unlocked=True,
        coil_1_on=True,
    )
    assert command_codes_for_transition(previous, target) == (
        CommandCode.COIL_1_ON,
    )


def test_unlock_without_device_changes_sends_nothing():
    """Changing only the local lock state does not produce a CAN command."""
    previous = MechanicalState(controls_unlocked=False)
    target = MechanicalState(controls_unlocked=True)
    assert command_codes_for_transition(previous, target) == ()


def test_watchdog_only_turns_off_active_devices():
    """Stop active outputs on timeout without resending unrelated states."""
    previous = MechanicalState(
        controls_unlocked=True,
        valve_1_on=True,
        coil_1_on=True,
    )
    assert command_codes_for_transition(
        previous, WATCHDOG_TIMEOUT_STATE
    ) == (
        CommandCode.VALVE_1_OFF,
        CommandCode.COIL_1_OFF,
    )
