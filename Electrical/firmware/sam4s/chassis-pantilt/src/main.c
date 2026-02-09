#include <asf.h>
#include "modbus_uart1.h"
#include "servo.h"

#define MODBUS_SLAVE_ID 2
#define MODBUS_BPS 115200
#define MODBUS_SER_PORT UART1
#define MODBUS_EN_PORT PIOA
#define MODBUS_EN_PIN PIO_PA17

enum MODBUS_REGISTERS {
	CENTER_ALL = 0,
	PAN_ADJUST_POSITIVE = 1,
	PAN_ADJUST_NEGATIVE = 2,
	TILT_ADJUST_POSITIVE = 3,
	TILT_ADJUST_NEGATIVE = 4,
};

// Pan/tilt hard limits
int pan_min = 530;
int pan_center = 1590;
int pan_max = 2460;
int tilt_min = 620;
int tilt_center = 1670;
int tilt_max = 2380;


static void board_setup(void) {
	WDT->WDT_MR |= WDT_MR_WDDIS; // Disable watchdog timer to prevent uC resetting every 15 seconds :)
}

static void handle_pan_tilt(servo_s *pan_servo, servo_s *tilt_servo) {
	if (intRegisters[CENTER_ALL]) {
		servo_write_us(pan_servo, pan_servo->us_center);
		servo_write_us(tilt_servo, tilt_servo->us_center);
		
		intRegisters[CENTER_ALL] = 0;
	} else {
		unsigned pan_pos = pan_servo->position + intRegisters[PAN_ADJUST_POSITIVE] - intRegisters[PAN_ADJUST_NEGATIVE];
		unsigned tilt_pos = tilt_servo->position + intRegisters[TILT_ADJUST_POSITIVE] - intRegisters[TILT_ADJUST_NEGATIVE];
		servo_write_us(pan_servo, pan_pos);
		servo_write_us(tilt_servo, tilt_pos);
		
		intRegisters[PAN_ADJUST_POSITIVE] = 0;
		intRegisters[PAN_ADJUST_NEGATIVE] = 0;
		intRegisters[TILT_ADJUST_POSITIVE] = 0;
		intRegisters[TILT_ADJUST_NEGATIVE] = 0;
	}
}

int main(void) {
	sysclk_init();
	board_setup();
	
	modbus_init(MODBUS_SLAVE_ID, MODBUS_SER_PORT, MODBUS_BPS, MODBUS_EN_PORT, MODBUS_EN_PIN);
	
	servo_s pan_servo;
	servo_s tilt_servo;
	//servo_s hitch_servo;
	
	servo_setup(&pan_servo, PWM_CHANNEL_0, pan_min, pan_max, pan_center);
	servo_setup(&tilt_servo, PWM_CHANNEL_1, tilt_min, tilt_max, tilt_center);
	//servo_setup(&tilt_servo, PWM_CHANNEL_2, 69, 69, 69); // CHANGE ME
	
	while (1) {
		modbus_update();
		handle_pan_tilt(&pan_servo, &tilt_servo);
		//handle_hitch(&hitch_servo);
	}
}
