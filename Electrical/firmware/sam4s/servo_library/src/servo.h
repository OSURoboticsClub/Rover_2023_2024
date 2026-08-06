#ifndef SERVO_H
#define SERVO_H

#include <asf.h>


typedef struct servo_s {
	// The PWM channel the servo is attached to
	uint32_t pwm_channel_num;
	pwm_channel_t pwm_channel;
	
	unsigned position;	// The current position of the servo (in us)
	unsigned us_min;	// The minimum position the servo can be in (in us)
	unsigned us_max;	// The maximum position the servo can be in (in us)
	unsigned us_center; // The center position of the servo can be in (in us)
} servo_s;


void servo_setup(servo_s *servo, uint32_t pwm_channel_num, unsigned us_min, unsigned us_max, unsigned us_center);
void servo_write_us(servo_s *servo, unsigned us);
void servo_write_angle(servo_s *servo, unsigned angle);


#endif
