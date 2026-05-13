#ifndef MODBUS_INTERFACE_H
#define MODBUS_INTERFACE_H

#include <stdint.h>

#define REGISTER_AR_SIZE	256		//Size of the register array for a given data type
extern uint16_t timeout;

extern uint16_t		intRegisters[REGISTER_AR_SIZE];
extern float		floatRegisters[REGISTER_AR_SIZE];
extern char			charRegisters[REGISTER_AR_SIZE];
extern bool			boolRegisters[REGISTER_AR_SIZE];

#ifdef __cplusplus
extern "C"
{
#endif

void serial_port_write(uint8_t *, uint16_t);
uint32_t get_elapsed_ms();

void modbus_init(uint8_t, uint8_t, uint8_t, const uint32_t, const uint16_t);
void modbus_update();
bool modbus_comm_good();

#ifdef __cplusplus
}
#endif

#endif
