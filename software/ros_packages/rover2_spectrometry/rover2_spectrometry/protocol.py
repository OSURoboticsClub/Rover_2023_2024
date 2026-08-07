"""Pure state-to-CAN protocol helpers for the science mechanisms."""

from dataclasses import dataclass
from enum import IntEnum


class CommandCode(IntEnum):
    """Science mechanism command IDs implemented by the MCU firmware."""

    VALVE_1_ON = 9
    VALVE_1_OFF = 10
    VALVE_2_ON = 11
    VALVE_2_OFF = 12
    ALL_VALVES_ON = 13
    ALL_VALVES_OFF = 14
    PUMP_ON = 15
    PUMP_OFF = 16
    COIL_1_ON = 17
    COIL_1_OFF = 18
    COIL_2_ON = 19
    COIL_2_OFF = 20
    ALL_COILS_ON = 21
    ALL_COILS_OFF = 22
    SOLENOID_ON = 23
    SOLENOID_OFF = 24


@dataclass(frozen=True)
class MechanicalState:
    """Complete desired or last-commanded state of the science mechanisms."""

    controls_unlocked: bool = False
    valve_1_on: bool = False
    valve_2_on: bool = False
    pump_on: bool = False
    coil_1_on: bool = False
    coil_2_on: bool = False
    solenoid_on: bool = False

    def normalized(self):
        """Force all outputs off whenever master control is locked."""
        if self.controls_unlocked:
            return self
        return MechanicalState()


# A heartbeat timeout turns every output off without engaging the master lock,
# so commands can resume immediately when communication returns.
WATCHDOG_TIMEOUT_STATE = MechanicalState(controls_unlocked=True)


def command_codes_for_state(state):
    """Return an idempotent CAN command sequence for one complete state."""
    state = state.normalized()
    if not state.controls_unlocked:
        # Locking is implemented by explicitly converging every output to off.
        return (
            CommandCode.VALVE_1_OFF,
            CommandCode.VALVE_2_OFF,
            CommandCode.PUMP_OFF,
            CommandCode.COIL_1_OFF,
            CommandCode.COIL_2_OFF,
            CommandCode.SOLENOID_OFF,
        )

    commands = []
    commands.extend(
        _paired_commands(
            state.valve_1_on,
            state.valve_2_on,
            CommandCode.ALL_VALVES_ON,
            CommandCode.ALL_VALVES_OFF,
            CommandCode.VALVE_1_ON,
            CommandCode.VALVE_1_OFF,
            CommandCode.VALVE_2_ON,
            CommandCode.VALVE_2_OFF,
        )
    )
    commands.append(
        CommandCode.PUMP_ON if state.pump_on else CommandCode.PUMP_OFF
    )
    commands.extend(
        _paired_commands(
            state.coil_1_on,
            state.coil_2_on,
            CommandCode.ALL_COILS_ON,
            CommandCode.ALL_COILS_OFF,
            CommandCode.COIL_1_ON,
            CommandCode.COIL_1_OFF,
            CommandCode.COIL_2_ON,
            CommandCode.COIL_2_OFF,
        )
    )
    commands.append(
        CommandCode.SOLENOID_ON
        if state.solenoid_on
        else CommandCode.SOLENOID_OFF
    )
    return tuple(commands)


def command_codes_for_transition(previous_state, target_state):
    """
    Return only the commands needed to transition between two states.

    If the previous MCU state is unknown, return a complete state sequence so
    startup explicitly converges every output to the requested state.
    """
    target_state = target_state.normalized()
    if previous_state is None:
        return command_codes_for_state(target_state)

    previous_state = previous_state.normalized()
    if previous_state == target_state:
        return ()

    if not target_state.controls_unlocked:
        commands = []
        if previous_state.valve_1_on:
            commands.append(CommandCode.VALVE_1_OFF)
        if previous_state.valve_2_on:
            commands.append(CommandCode.VALVE_2_OFF)
        if previous_state.pump_on:
            commands.append(CommandCode.PUMP_OFF)
        if previous_state.coil_1_on:
            commands.append(CommandCode.COIL_1_OFF)
        if previous_state.coil_2_on:
            commands.append(CommandCode.COIL_2_OFF)
        if previous_state.solenoid_on:
            commands.append(CommandCode.SOLENOID_OFF)
        return tuple(commands)

    commands = []
    commands.extend(
        _paired_transition_commands(
            previous_state.valve_1_on,
            previous_state.valve_2_on,
            target_state.valve_1_on,
            target_state.valve_2_on,
            CommandCode.ALL_VALVES_ON,
            CommandCode.ALL_VALVES_OFF,
            CommandCode.VALVE_1_ON,
            CommandCode.VALVE_1_OFF,
            CommandCode.VALVE_2_ON,
            CommandCode.VALVE_2_OFF,
        )
    )
    if previous_state.pump_on != target_state.pump_on:
        commands.append(
            CommandCode.PUMP_ON
            if target_state.pump_on
            else CommandCode.PUMP_OFF
        )
    commands.extend(
        _paired_transition_commands(
            previous_state.coil_1_on,
            previous_state.coil_2_on,
            target_state.coil_1_on,
            target_state.coil_2_on,
            CommandCode.ALL_COILS_ON,
            CommandCode.ALL_COILS_OFF,
            CommandCode.COIL_1_ON,
            CommandCode.COIL_1_OFF,
            CommandCode.COIL_2_ON,
            CommandCode.COIL_2_OFF,
        )
    )
    if previous_state.solenoid_on != target_state.solenoid_on:
        commands.append(
            CommandCode.SOLENOID_ON
            if target_state.solenoid_on
            else CommandCode.SOLENOID_OFF
        )
    return tuple(commands)


def _paired_commands(
    first_on,
    second_on,
    all_on_command,
    all_off_command,
    first_on_command,
    first_off_command,
    second_on_command,
    second_off_command,
):
    if first_on == second_on:
        return (all_on_command if first_on else all_off_command,)
    return (
        first_on_command if first_on else first_off_command,
        second_on_command if second_on else second_off_command,
    )


def _paired_transition_commands(
    previous_first_on,
    previous_second_on,
    target_first_on,
    target_second_on,
    all_on_command,
    all_off_command,
    first_on_command,
    first_off_command,
    second_on_command,
    second_off_command,
):
    first_changed = previous_first_on != target_first_on
    second_changed = previous_second_on != target_second_on
    if not first_changed and not second_changed:
        return ()
    both_changed_to_same_state = (
        first_changed
        and second_changed
        and target_first_on == target_second_on
    )
    if both_changed_to_same_state:
        return (all_on_command if target_first_on else all_off_command,)

    commands = []
    if first_changed:
        commands.append(
            first_on_command if target_first_on else first_off_command
        )
    if second_changed:
        commands.append(
            second_on_command if target_second_on else second_off_command
        )
    return tuple(commands)
