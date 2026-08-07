"""Unit tests for science mechanism state-to-CAN translation."""

from spectrometry_mechanical.protocol import CommandCode
from spectrometry_mechanical.protocol import MechanicalState
from spectrometry_mechanical.protocol import WATCHDOG_TIMEOUT_STATE
from spectrometry_mechanical.protocol import command_codes_for_state


def test_locked_state_locks_before_redundant_output_off_commands():
    state = MechanicalState(
        controls_unlocked=False,
        valve_1_on=True,
        valve_2_on=True,
        pump_on=True,
        coil_1_on=True,
        coil_2_on=True,
        solenoid_on=True,
    )

    assert command_codes_for_state(state) == (
        CommandCode.LOCK_SCIENCE,
        CommandCode.ALL_VALVES_OFF,
        CommandCode.PUMP_OFF,
        CommandCode.ALL_COILS_OFF,
        CommandCode.SOLENOID_OFF,
    )


def test_unlocked_mixed_state_uses_expected_firmware_commands():
    state = MechanicalState(
        controls_unlocked=True,
        valve_1_on=True,
        valve_2_on=False,
        pump_on=True,
        coil_1_on=False,
        coil_2_on=True,
        solenoid_on=True,
    )

    assert command_codes_for_state(state) == (
        CommandCode.UNLOCK_SCIENCE,
        CommandCode.VALVE_1_ON,
        CommandCode.VALVE_2_OFF,
        CommandCode.PUMP_ON,
        CommandCode.COIL_1_OFF,
        CommandCode.COIL_2_ON,
        CommandCode.SOLENOID_ON,
    )


def test_watchdog_state_turns_everything_off_without_locking():
    assert command_codes_for_state(WATCHDOG_TIMEOUT_STATE) == (
        CommandCode.UNLOCK_SCIENCE,
        CommandCode.ALL_VALVES_OFF,
        CommandCode.PUMP_OFF,
        CommandCode.ALL_COILS_OFF,
        CommandCode.SOLENOID_OFF,
    )


def test_group_commands_are_used_when_paired_outputs_match():
    state = MechanicalState(
        controls_unlocked=True,
        valve_1_on=True,
        valve_2_on=True,
        coil_1_on=False,
        coil_2_on=False,
    )

    commands = command_codes_for_state(state)

    assert CommandCode.ALL_VALVES_ON in commands
    assert CommandCode.ALL_COILS_OFF in commands
    assert CommandCode.VALVE_1_ON not in commands
    assert CommandCode.COIL_1_OFF not in commands
