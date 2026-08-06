#include "stepper.h"

/* The higher this is, the more accurate the PWM.
 * However, the higher it is means the lower max frequency supported due to overflow.
 */

// Percentage of duty cycle, ie 0.5 = 50% duty cycle
#define PWM_DUTY_CYCLE 0.5
#define PWM_CLK_SPEED 10000
#define STEP_FREQ 100


static void _init_pwm(stepper_s *stepper) {
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, stepper->pwm_channel_num);
	
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_CLK_SPEED,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_bus_hz(PWM)
	};
	pwm_init(PWM, &clock_setting);
}

static void _pwm_set_freq(stepper_s *stepper, unsigned freq) {
	int period = PWM_CLK_SPEED / (freq <= PWM_CLK_SPEED ? freq : PWM_CLK_SPEED);
	
	pwm_channel_t pwm_channel_instance = {
		.ul_prescaler = PWM_CMR_CPRE_CLKA,
		.ul_period = period,
		.ul_duty = period * PWM_DUTY_CYCLE,
		.channel = stepper->pwm_channel_num
	};
	stepper->pwm_channel = pwm_channel_instance;
	
	pwm_channel_disable(PWM, stepper->pwm_channel_num);
	pwm_channel_init(PWM, &stepper->pwm_channel);
	pwm_channel_enable(PWM, stepper->pwm_channel_num);
}

void stepper_setup(stepper_s *stepper, uint32_t pwm_channel_num, Pio *dir_port, uint32_t dir_pin, Pio *step_port, uint32_t step_pin) {
	stepper->position = 0;
	
	stepper->dir_port = dir_port;
	stepper->dir_pin = dir_pin;
	stepper->step_port = step_port;
	stepper->step_pin = step_pin;
	stepper->pwm_channel_num = pwm_channel_num;
	
	pio_set_output(dir_port, dir_pin, HIGH, DISABLE, DISABLE);
	pio_set_peripheral(PIOA, PIO_PERIPH_B, step_pin);
	_init_pwm(stepper);
}

void stepper_set_velocity(stepper_s *stepper, unsigned steps_per_sec, const stepper_dir dir) {
	if (dir == STEPPER_DIR_CW) {
		pio_set(stepper->dir_port, stepper->dir_pin);
	} else {
		pio_clear(stepper->dir_port, stepper->dir_pin);
	}
	
	_pwm_set_freq(stepper, steps_per_sec);
}

void stepper_set_position(stepper_s *stepper, int pos){
	int delta = pos - stepper->position;			//find out how we need to move
	
	if(delta <= 0) pio_set(stepper->dir_port, stepper->dir_pin);		//sets direction positive or negative.
	else pio_clear(stepper->dir_port, stepper->dir_pin);
	
	delta = delta > 0? delta : -delta;				//take absolute value of delta
	
	int period = PWM_CLK_SPEED / STEP_FREQ;
	pwm_channel_t pwm_channel_instance = {
		.ul_prescaler = PWM_CMR_CPRE_CLKA,
		.ul_period = period,
		.ul_duty = period * PWM_DUTY_CYCLE,
		.channel = stepper->pwm_channel_num
	};
	stepper->pwm_channel = pwm_channel_instance;
	
	pwm_channel_disable(PWM, stepper->pwm_channel_num);
	pwm_channel_init(PWM, &stepper->pwm_channel);
	pwm_channel_enable(PWM, stepper->pwm_channel_num);
	
	for (int i=0; i<delta; i++){					//make a step for each step that needs to be made. then disable PWM
		while(!PWM->PWM_ISR1){}	//Wait until counter event occurs (on counter reset) Reading this register clears it, !! Double check that the register is actually clearing. Reference page 961 of the datasheet
	}												//I chose the blocking method because dealing with interrupts without testing is hard.
	//for (volatile uint32_t i = 0; i < (12000000) * 3; i++);
	pwm_channel_disable(PWM, stepper->pwm_channel_num);
	
	stepper->position = pos;
}

void stepper_stop(stepper_s *stepper) {
	pwm_channel_disable(PWM, stepper->pwm_channel_num);
}