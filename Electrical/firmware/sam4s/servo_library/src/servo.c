/* This is a very basic servo library meant to control typical servo motors used by the Rover.
 * Servo motors typically use PWM signals to determine which position to rotate to.
 * Most servo motors expect a pulse (signal set HIGH) every 20ms (a frequency of 50Hz) in order to operate.
 * The length of this pulse determines the position of the motor.
 * The library works by being given pulse widths in microseconds to generate the correct PWM signal.
 * The duty cycle (which is the percentage of the period that the signal is HIGH) of the PWM signal is calculated based on this given pulse width. */
 
#include "servo.h"

/* Defines the speed the PWM module runs at.
 * The higher the number, the more accurate (aka higher resolution) of the PWM signal.
 * Unfortunately the register that tracks each tick for duty cycle purposes is 16-bit (thus max value of 65,535).
 * This means that if the given frequency is low enough for a given clock speed, the register will overflow causing all sorts of problems.
 * So since we are using a PWM frequency of 50Hz, at a clock speed of 1,000,000Hz, our period is 20,000 ticks.
 * Since servo pulses are usually 500 - 2,500 us in length, the duty cycle is typically 17,500 - 19,500 us in length (or 17,500 - 19,500 ticks) which fits in the register.
 * If we had a clock speed of, say, 4,000,000Hz, the period would quadruple in ticks and the duty cycles would typically be 70,000 - 78,000 ticks which would overflow the register.
 * The point is, be mindful of what you set the clock speed to. */
#define PWM_CLK_SPEED 1000000

// Servo motors expect a pulse every 20ms (20,0000 us) which is a frequency of 50Hz
#define PWM_FREQ 50

// Represents the clock ticks for a single PWM period (which should be 20ms (20,000 us))
#define PWM_PERIOD (PWM_CLK_SPEED / PWM_FREQ)

// Used to convert duty cycles given in microseconds to clock ticks
#define MICROSECONDS 1000000.0

static bool pwm_initiated = false;

/* Initializes the PWM module */
static void _init_pwm(servo_s *servo) {
	if (!pwm_initiated) {
		pmc_enable_periph_clk(ID_PWM);
		
		pwm_clock_t clock_setting = {
			.ul_clka = PWM_CLK_SPEED,
			.ul_clkb = 0,
			.ul_mck = sysclk_get_peripheral_bus_hz(PWM)
		};
		pwm_init(PWM, &clock_setting);
		pwm_initiated = true;
	}
}

/* Sets the duty cycle (given in microseconds) of the PWM channel of the given servo */
static void _pwm_set_duty(servo_s *servo, unsigned duty_us) {
	// Duty cycle is given in microseconds so convert to actual clock ticks
	unsigned duty_ticks = (PWM_CLK_SPEED / MICROSECONDS) * duty_us;
	
	pwm_channel_t pwm_channel_instance = {
		.ul_prescaler = PWM_CMR_CPRE_CLKA,
		.ul_period = PWM_PERIOD,
		.ul_duty = PWM_PERIOD - duty_ticks, // Basically, how many clock ticks PWM signal should be low (kind of the inverse of duty cycle)
		.channel = servo->pwm_channel_num
	};
	servo->pwm_channel = pwm_channel_instance;
	
	// Have to disable then re-enable PWM channel for changes to take effect
	pwm_channel_disable(PWM, servo->pwm_channel_num);
	pwm_channel_init(PWM, &servo->pwm_channel);
	pwm_channel_enable(PWM, servo->pwm_channel_num);
}

/* Sets up the PWN channel for a given servo */
void servo_setup(servo_s *servo, uint32_t pwm_channel_num, unsigned us_min, unsigned us_max, unsigned us_center) {
	servo->pwm_channel_num = pwm_channel_num;
	servo->us_min = us_min;
	servo->us_max = us_max;
	servo->us_center = us_center;
	
	/* SAM4S boards have 4 different PWM channels corresponding to different pins.
	 * If a bad channel is given just return and don't initialize PWM. */
	uint32_t pwm_pin;
	switch (pwm_channel_num) {
		case PWM_CHANNEL_0:
			pwm_pin = PIO_PA11;
			break;
		case PWM_CHANNEL_1:
			pwm_pin = PIO_PA12;
			break;
		case PWM_CHANNEL_2:
			pwm_pin = PIO_PA13;
			break;
		case PWM_CHANNEL_3:
			pwm_pin = PIO_PA7;
			break;
		default:
			return;
	}
	
	// Needed so that the MCU knows the given pin is controlled by the PWM module.
	pio_set_peripheral(PIOA, PIO_PERIPH_B, pwm_pin);
	
	// Initializes PWM module if it hasn't been already
	_init_pwm(servo);
	
	// Set the servo to its center position on power-up
	servo->position = us_center;
	_pwm_set_duty(servo, us_center);
}

/* Adjusts the duty cycle of PWM based on the given duty cycle in microseconds */
void servo_write_us(servo_s *servo, unsigned us) {
	// Keep angle constrained to min/max limits
	if (us > servo->us_max) {
		us = servo->us_max;
	} else if (us < servo->us_min) {
		us = servo->us_min;
	}
	
	_pwm_set_duty(servo, us);
	servo->position = us;
}

/* Converts an angle into the microseconds of the corresponding duty cycle */
void servo_write_angle(servo_s *servo, unsigned angle) {
	int us = (angle * (servo->us_max - servo->us_min) / 180) + servo->us_min;
	servo_write_us(servo, us);
}
