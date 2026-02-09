#ifndef MODBUS_INTERFACE_H
#define MODBUS_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <asf.h>

// Referenced by slave library and nodes
#define REGISTER_AR_SIZE	256		//Size of the register array for a given data type
extern uint16_t timeout;
extern uint32_t elapsed_ms;

// Referenced by slave library (might one day factor out into separate library)
void serial_port_write(uint8_t*, uint16_t);
uint32_t get_elapsed_ms(void);

// Referenced by nodes
extern uint16_t		intRegisters[REGISTER_AR_SIZE];
extern float		floatRegisters[REGISTER_AR_SIZE];
extern char			charRegisters[REGISTER_AR_SIZE];
extern bool			boolRegisters[REGISTER_AR_SIZE];

void modbus_init(int, Uart*, const uint32_t, Pio*, const uint32_t);
void modbus_update(void);
void modbus_timeout_en(bool enable, uint16_t timeout_ms);
void modbus_set_rx_en_pin(uint32_t rx_en_pin);
bool modbus_comm_good(void);

void UART_Handler(void);

#endif
