# Rover-side command/CAN status. Output fields are the last state successfully
# sent to SocketCAN; they are not hardware-confirmed until MCU telemetry exists.
uint32 command_sequence
bool command_link_active
bool can_connected
bool command_state_valid
bool controls_unlocked
bool valve_1_on
bool valve_2_on
bool pump_on
bool coil_1_on
bool coil_2_on
