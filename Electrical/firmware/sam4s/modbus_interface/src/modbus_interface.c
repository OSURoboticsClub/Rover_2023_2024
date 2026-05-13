#include "modbus_interface.h"
#include "modbus.h"

/* Begin modbus slave library AND node references */
static bool timeout_active = false;
uint16_t timeout;
uint32_t elapsed_ms = 0;
/* End modbus slave library AND node references */

/* Begin modbus slave library references */

Uart *RS485Port;
Pio *globalEnPinPort;
uint32_t globalEnPin;
uint32_t rxEnPin = 0;

uint16_t transmitIndex; // helper variables for transmitting

static void init_timer(void) {
	sysclk_enable_peripheral_clock(ID_TC0);
	NVIC_EnableIRQ(TC0_IRQn);
	
	uint32_t mode =	TC_CMR_WAVE					// Waveform mode
				  | TC_CMR_TCCLKS_TIMER_CLOCK5;	// 32kHz slow clock (since only need ms accuracy)
	
	tc_init(TC0, 0, mode);
	tc_enable_interrupt(TC0, 0, TC_IER_COVFS);
	tc_start(TC0, 0);
	
}

void serial_port_write(uint8_t *packet, uint16_t packetSize) {
	pio_set(globalEnPinPort, globalEnPin); // transceiver transmit enable
	if (rxEnPin > 0) {
		pio_set(globalEnPinPort, rxEnPin); // receiver disable
	}
	
	transmitIndex = 0;
	uart_enable_interrupt(RS485Port, UART_IMR_TXRDY);
}

uint32_t get_elapsed_ms(void) {
	// Return elapsed ms plus the current value of the timer
	// Since the timer ticks 32000 times a second, divide by 32 to get ms
	return timeout_active ? elapsed_ms + (tc_read_cv(TC0, 0) / 32) : 0;
}
/* End modbus slave library references */

/* Begin node references */
void modbus_init(int slave_id, Uart *port485, const uint32_t baud, Pio *enPinPort, const uint32_t enPin) {
	RS485Port = port485;

	if (RS485Port == UART0) {
		pmc_enable_periph_clk(ID_UART0);                  // Enable the clocks to the UART modules
		pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA9);  // Sets PA9 to RX
		pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA10); // Sets PA10 to TX
		NVIC_EnableIRQ(UART0_IRQn);                       // enables interrupts related to this port
	}

	if (RS485Port == UART1) {
		pmc_enable_periph_clk(ID_UART1);                 // Enable the clocks to the UART modules
		pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB2); // Sets PB2 to RX
		pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB3); // Sets PB3 to TX
		NVIC_EnableIRQ(UART1_IRQn);                      // enables interrupts related to this port
	}

	uint32_t clockSpeed = sysclk_get_peripheral_bus_hz(RS485Port); // gets CPU speed to for baud counter

	sam_uart_opt_t UARTSettings = {
		.ul_baudrate = baud,                               // sets baudrate
		.ul_mode = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO, // sets to normal mode
		.ul_mck = clockSpeed                               // sets baud counter clock
	};

	uart_init(RS485Port, &UARTSettings); // init the UART port
	uart_enable_rx(RS485Port);
	uart_enable_tx(RS485Port);
	uart_enable_interrupt(RS485Port, UART_IER_RXRDY); // Enable interrupt for incoming data

	pio_set_output(enPinPort, enPin, LOW, DISABLE, DISABLE); // init the enable pin
	globalEnPinPort = enPinPort;
	globalEnPin = enPin;
	
	modbus_slave_init(slave_id);
}

void modbus_update(void) {
	modbus_slave_update();
}

void modbus_timeout_en(bool enable, uint16_t timeout_ms) {
	if (enable) {
		elapsed_ms = 0;
		timeout = timeout_ms;
		init_timer();
		timeout_active = true;
	} else {
		tc_stop(TC0, 0);
		timeout_active = false;
	}
}

void modbus_set_rx_en_pin(uint32_t rx_en_pin) {
	pio_set_output(globalEnPinPort, rx_en_pin, LOW, DISABLE, DISABLE);
	rxEnPin = rx_en_pin;
}

bool modbus_comm_good(void) {
	return timeout_active ? modbus_slave_comm_good() : true;
}

// interrupt handler for incoming data
void UART_Handler(void) {
	if (uart_is_rx_ready(RS485Port)) {                         // confirm there is data ready to be read
		uart_read(RS485Port, &(rxBuffer.data[rxBuffer.head])); // move the data into the next index of the rx buffer
		rxBuffer.head = PKT_WRAP_ARND(rxBuffer.head + 1);      // iterate the head through the ring buffer
	} else if (uart_is_tx_ready(RS485Port)) {
		if (transmitIndex < responsePacketSize) {
			uart_write(RS485Port, responsePacket[transmitIndex]);
			transmitIndex++;
		} else if (uart_is_tx_empty(RS485Port)) {
			pio_clear(globalEnPinPort, globalEnPin);
			if (rxEnPin > 0) {
				pio_clear(globalEnPinPort, rxEnPin); // receiver enable
			}
			
			uart_disable_interrupt(RS485Port, UART_IMR_TXRDY);
		}
	}
}
/* End node references */