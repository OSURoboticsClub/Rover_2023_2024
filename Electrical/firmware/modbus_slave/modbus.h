/*
 * IncFile1.h
 *
 * Created: 2/3/2022 3:37:46 PM
 *  Author: Anthony Grana,
			Blake Hakkila,
			Kurtis Dinelle
 */


/**********************************************************************
*REQUIREMENTS
***********************************************************************
*In ASF Wizard include UART (not USART)
*(Debuging only) in ASF Wizard include USB Device in CDC mode and run udc_start(); in init.c
*/

/**********************************************************************
*
*	HOW THIS MODBUS LIBRARY WORKS
*
*this is a SLAVE ONLY interrupt based modbus library that supports the following function codes:
*0x03 Read multiple holding registers
*0x10 Write multiple holding registers
*
*This library supports a non-standard extended register range up 0-1023 with different data types
*0-255		Int Registers
*256-511	Float Registers
*512-767	Char Registers
*768-1023	Bool Registers
*
*This structure was specifically chosen to maintain backwards compatibility with the rover's systems from 2017-2022
*
*	INSTRUCTIONS
*inside of init.c run the modbus_init(); function
*Inside of a fast opperating main loop run the function modbus_update();
***********************************************************************/

#include <stdint.h>
#include <stdbool.h>

#ifndef MODBUS_H_
#define MODBUS_H_

#define RX_BUFFER_SIZE 8192 // size of RX buffer, this determines max incoming packet size (MUST BE A POWER OF 2)
#define TX_BUFFER_SIZE 8192 // !!!!!! Important change this value to the size of the tx buffer in the UART module

// To handle edge case if a packet starts toward end of buffer but wraps around to beginning
#define PKT_WRAP_ARND(idx) \
((idx) & (RX_BUFFER_SIZE - 1))

//#define MODBUS_DEBUG				//uncomment this to enable debugging over USB_CDC this depends on USB_CDC being initialized elsewhere

#ifdef __cplusplus
extern "C" {
#endif

struct ringBuffer
{                  // ring buffer to serve as rx buffer. Helps solve lots of data shifting problems
    uint16_t head; // Next free space in the buffer
    uint16_t tail; // Start of unprocessed data and beginning of packet
    uint8_t data[RX_BUFFER_SIZE];
};

extern struct ringBuffer rxBuffer;
extern uint8_t responsePacket[TX_BUFFER_SIZE];
extern uint16_t responsePacketSize;

void modbus_slave_init(const uint8_t);    // Initialize modbus uart port, clock, memory, transmit enable, and ...
void modbus_slave_update(void);   //This function does all of the heavy lifting for modbus
bool modbus_slave_comm_good(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H_ */