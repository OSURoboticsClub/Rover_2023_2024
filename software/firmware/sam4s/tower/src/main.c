#include <asf.h>
#include "modbus_uart1.h"

// modbus definitions
#define MODBUS_SLAVE_ID 1
#define MODBUS_BPS 115200
#define MODBUS_SER_PORT UART1
#define MODBUS_EN_PORT PIOB
#define MODBUS_EN_PIN PIO_PB1

enum MODBUS_REGISTERS {
	LIGHT = 0,
	ROVER_LAT_SIGN = 1,
	ROVER_LAT_DEG = 2,
	ROVER_LAT_FRAC = 3,
	ROVER_LONG_SIGN = 4,
	ROVER_LONG_DEG = 5,
	ROVER_LONG_FRAC = 6,
	ASTRONAUT_LAT_SIGN = 7,
	ASTRONAUT_LAT_DEG = 8,
	ASTRONAUT_LAT_FRAC = 9,
	ASTRONAUT_LONG_SIGN = 10,
	ASTRONAUT_LONG_DEG = 11,
	ASTRONAUT_LONG_FRAC = 12
};

// constants for getting gps data
#define GPS_MAGIC 255
#define SIGN_OFFSET 0
#define DEGREE_OFFSET 1
#define FRAC_OFFSET 2
#define FRAC_HIGH_OFFSET 2
#define FRAC_LOW_OFFSET 3
#define GPS_COORD_LEN 4
#define NUM_COORDS 4
#define GPS_PKT_SIZE GPS_COORD_LEN * NUM_COORDS + 2
#define REG_PER_COORD 3
#define COORD_REG_OFFSET 1

// gps serial definitions
#define GPS_BPS 115200
#define GPS_SER_PORT UART0
#define GPS_SER_ID ID_UART0
#define GPS_SER_INTERRUPT UART0_IRQn
#define GPS_SER_PERIPHERAL PIO_PERIPH_A
#define GPS_PIN_PORT PIOA
#define GPS_EN_PIN PIO_PA8
#define GPS_RX_PIN PIO_PA9
#define GPS_TX_PIN PIO_PA10

// gps buffer definitions
#define GPS_BUFFER_SIZE 1024 // must be power of 2
struct
{                  // ring buffer to serve as rx buffer. Helps solve lots of data shifting problems
	uint16_t head; // Next free space in the buffer
	uint16_t tail; // Start of unprocessed data and beginning of packet
	uint8_t data[GPS_BUFFER_SIZE];
} gps_buffer;
#define GPS_WRAP_ARND(idx) \
((idx) & (GPS_BUFFER_SIZE - 1))

// pin io definitions
#define GPIO_PORT PIOA
#define TEST_LED PIO_PA3
#define IR_LIGHT PIO_PA4
#define VISUAL_LIGHT PIO_PA5
#define IR 2
#define VISUAL 1

static void init_gps_uart(void) {
	pmc_enable_periph_clk(GPS_SER_ID);                                 // Enable the clocks to the UART modules
	pio_set_peripheral(GPS_PIN_PORT, GPS_SER_PERIPHERAL, GPS_RX_PIN);  // Sets PA9 to RX
	pio_set_peripheral(GPS_PIN_PORT, GPS_SER_PERIPHERAL, GPS_TX_PIN);  // Sets PA10 to TX
	NVIC_EnableIRQ(GPS_SER_INTERRUPT);                                 // enables interrupts related to this port

	uint32_t clockSpeed = sysclk_get_peripheral_bus_hz(GPS_SER_PORT); // gets CPU speed to for baud counter

	sam_uart_opt_t UARTSettings = {
		.ul_baudrate = GPS_BPS,                               // sets baudrate
		.ul_mode = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO, // sets to normal mode
		.ul_mck = clockSpeed                               // sets baud counter clock
	};

	uart_init(GPS_SER_PORT, &UARTSettings); // init the UART port
	uart_enable_rx(GPS_SER_PORT);
	uart_enable_tx(GPS_SER_PORT);
	uart_enable_interrupt(GPS_SER_PORT, UART_IER_RXRDY); // Enable interrupt for incoming data

	pio_set_output(GPS_PIN_PORT, GPS_EN_PIN, LOW, DISABLE, DISABLE); // init the enable pin
}

void UART0_Handler() {
	if (uart_is_rx_ready(GPS_SER_PORT)) {                         // confirm there is data ready to be read
		uart_read(GPS_SER_PORT, &(gps_buffer.data[gps_buffer.head])); // move the data into the next index of the rx buffer
		gps_buffer.head = GPS_WRAP_ARND(gps_buffer.head + 1);      // iterate the head through the ring buffer
	}
}

static void board_setup(void) {
	WDT->WDT_MR |= WDT_MR_WDDIS; // Disable watchdog timer to prevent uC resetting every 15 seconds :)
	pio_set_output(GPIO_PORT,TEST_LED,LOW,DISABLE,DISABLE);
	pio_set_output(GPIO_PORT,IR_LIGHT,LOW,DISABLE,DISABLE);
	pio_set_output(GPIO_PORT,VISUAL_LIGHT,LOW,DISABLE,DISABLE);
	init_gps_uart();
}

static void toggle_lights(void) {
	if (intRegisters[LIGHT] == VISUAL) {
		pio_set_output(GPIO_PORT,IR_LIGHT,LOW,DISABLE,DISABLE);
		pio_set_output(GPIO_PORT,VISUAL_LIGHT,HIGH,DISABLE,DISABLE);
	} else if (intRegisters[LIGHT] == IR) {
		pio_set_output(GPIO_PORT,IR_LIGHT,HIGH,DISABLE,DISABLE);
		pio_set_output(GPIO_PORT,VISUAL_LIGHT,LOW,DISABLE,DISABLE);
	} else {
		pio_set_output(GPIO_PORT,IR_LIGHT,LOW,DISABLE,DISABLE);
		pio_set_output(GPIO_PORT,VISUAL_LIGHT,LOW,DISABLE,DISABLE);
	}
}

static void update_gps(void) {
	int buffer_length = (gps_buffer.head - gps_buffer.tail + GPS_BUFFER_SIZE) & (GPS_BUFFER_SIZE - 1);
	while (buffer_length >= GPS_PKT_SIZE) {
		if (gps_buffer.data[gps_buffer.tail] == GPS_MAGIC && gps_buffer.data[GPS_WRAP_ARND(gps_buffer.tail + GPS_PKT_SIZE - 1)] == GPS_MAGIC) {
			break;
		} else {
			gps_buffer.tail = GPS_WRAP_ARND(gps_buffer.tail + 1);
			buffer_length--;
		}
	}
	if (buffer_length >= GPS_PKT_SIZE) {
		gps_buffer.tail = GPS_WRAP_ARND(gps_buffer.tail + 1); //remove magic byte
		for (int i = 0; i < NUM_COORDS; i++) {
			int base_index = REG_PER_COORD * i + COORD_REG_OFFSET;
			intRegisters[base_index + SIGN_OFFSET] = gps_buffer.data[GPS_WRAP_ARND(gps_buffer.tail + SIGN_OFFSET)]; // sign
			intRegisters[base_index + DEGREE_OFFSET] = gps_buffer.data[GPS_WRAP_ARND(gps_buffer.tail + DEGREE_OFFSET)]; // degree
			intRegisters[base_index + FRAC_OFFSET] = gps_buffer.data[GPS_WRAP_ARND(gps_buffer.tail + FRAC_HIGH_OFFSET)] << 8; // fraction high byte
			intRegisters[base_index + FRAC_OFFSET] += gps_buffer.data[GPS_WRAP_ARND(gps_buffer.tail + FRAC_LOW_OFFSET)]; // fraction low byte
			gps_buffer.tail = GPS_WRAP_ARND(gps_buffer.tail + GPS_COORD_LEN);
		}
		gps_buffer.tail = GPS_WRAP_ARND(gps_buffer.tail + 1); // remove magic byte
	}
}

int main(void) {
	sysclk_init();
	board_setup();
	
	modbus_init(MODBUS_SLAVE_ID, MODBUS_SER_PORT, MODBUS_BPS, MODBUS_EN_PORT, MODBUS_EN_PIN);
	
	while (1) {
		update_gps();
		modbus_update();
		toggle_lights();
	}
}
