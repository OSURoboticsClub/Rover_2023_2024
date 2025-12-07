#include <asf.h>
#include "modbus_uart0.h"
#include "stepper.h"

#define STEPPER_PWM_CHANNEL PWM_CHANNEL_3
#define STEPPER_DIR_PORT PIOA
#define STEPPER_DIR_PIN PIO_PA6
#define STEPPER_STEP_PORT PIOA
#define STEPPER_STEP_PIN PIO_PA7

// all floats for floating point math
#define STEPS_PER_MOTOR_REVOLUTION 200.0 // 1.8 degrees per step
#define MOTOR_GEAR_TEETH 28.0
#define COMPARTMENT_GEAR_TEETH 48.0
#define NUM_COMPARTMENTS 4.0

#define GPIO_PORT PIOA

#define LSW1_PIN				PIO_PA0
#define LSW2_PIN				PIO_PA1
#define LSW3_PIN				PIO_PA2
#define LSW4_PIN				PIO_PA3

#define LAZER_PIN				PIO_PA13
#define VID_SEL0_PIN			PIO_PA14
#define VID_SEL1_PIN			PIO_PA15

// modbus settings
#define MODBUS_SLAVE_ID 4
#define MODBUS_BPS 115200
#define MODBUS_SER_PORT UART0
#define MODBUS_EN_PORT PIOA
#define MODBUS_EN_PIN PIO_PA11

enum MODBUS_REGISTERS {
	COMPARTMENT_NUMBER = 0,
	CAMERA_SELECT = 1,
	LAZER_EN = 2,
	LIM_SW1 = 3,
	LIM_SW2 = 4,
	LIM_SW3 = 5,
	LIM_SW4 = 6
};

//int steps_per_rev = 465;
const float steps_per_compartment = (COMPARTMENT_GEAR_TEETH * STEPS_PER_MOTOR_REVOLUTION) / (MOTOR_GEAR_TEETH * NUM_COMPARTMENTS);
const int wraparound_position = (COMPARTMENT_GEAR_TEETH * STEPS_PER_MOTOR_REVOLUTION) / (MOTOR_GEAR_TEETH) + 0.5; // adding 0.5 will round to nearest int in conversion

static void board_setup(void) {
	WDT->WDT_MR |= WDT_MR_WDDIS; // Disable watchdog timer to prevent uC resetting every 15 seconds :)
	
	pmc_enable_periph_clk(ID_PIOA);			//This enables GPIO Output, (necessary)
	pmc_enable_periph_clk(ID_PIOB);
	
	pio_set_input(GPIO_PORT, LSW1_PIN, PIO_DEBOUNCE);
	pio_set_input(GPIO_PORT, LSW2_PIN, PIO_DEBOUNCE);
	pio_set_input(GPIO_PORT, LSW3_PIN, PIO_DEBOUNCE);
	pio_set_input(GPIO_PORT, LSW4_PIN, PIO_DEBOUNCE);
	
	pio_set_output(GPIO_PORT, LAZER_PIN, LOW, DISABLE, DISABLE);
	pio_set_output(GPIO_PORT, VID_SEL0_PIN, LOW, DISABLE, DISABLE);
	pio_set_output(GPIO_PORT, VID_SEL1_PIN, LOW, DISABLE, DISABLE);
}

int main(void) {
	sysclk_init();
	board_setup();
	
	modbus_init(MODBUS_SLAVE_ID, MODBUS_SER_PORT, MODBUS_BPS, MODBUS_EN_PORT, MODBUS_EN_PIN);
	
	stepper_s stepper;
	stepper_setup(&stepper, STEPPER_PWM_CHANNEL, STEPPER_DIR_PORT, STEPPER_DIR_PIN, STEPPER_STEP_PORT, STEPPER_STEP_PIN);

	while (1) {
		modbus_update();
		
		// only move in one direction for compartment change
		int compartment_position = steps_per_compartment * intRegisters[COMPARTMENT_NUMBER] + 0.5;
		if(compartment_position < stepper.position) {
			stepper_set_position(&stepper, wraparound_position); // first go back to first compartment and reset position, then go to compartment
			stepper.position = 0;
		}
		
		if (compartment_position != stepper.position) {
			stepper_set_position(&stepper, compartment_position);	//If the position needs to change, change it.
		}
		
		intRegisters[LIM_SW1] = pio_get(GPIO_PORT, PIO_TYPE_PIO_INPUT, LSW1_PIN);
		intRegisters[LIM_SW2] = pio_get(GPIO_PORT, PIO_TYPE_PIO_INPUT, LSW2_PIN);
		intRegisters[LIM_SW3] = pio_get(GPIO_PORT, PIO_TYPE_PIO_INPUT, LSW3_PIN);
		intRegisters[LIM_SW4] = pio_get(GPIO_PORT, PIO_TYPE_PIO_INPUT, LSW4_PIN);
		
		if(intRegisters[LAZER_EN]) {
			pio_set(GPIO_PORT, LAZER_PIN);
		} else{
			pio_clear(GPIO_PORT, LAZER_PIN);
		}
		
		
		if(intRegisters[CAMERA_SELECT]) {
			pio_set(GPIO_PORT, VID_SEL0_PIN);				//This is the only video select pin that matters
		} else {
			pio_clear(GPIO_PORT, VID_SEL1_PIN);
		}
	}
}
