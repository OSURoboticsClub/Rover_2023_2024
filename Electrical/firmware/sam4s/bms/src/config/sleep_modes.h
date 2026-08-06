/*
 * sleep_modes.h
 *
 * Created: 3/8/2022 10:37:53 PM
 *  Author: Anthony
 */ 


#ifndef SLEEP_MODES_H_
#define SLEEP_MODES_H_

#define PERIODIC_WAKEUP_TIME 10
#define RTT_DEFAULT_MODE 0

void USBWakeUp();
void PWRSwitchWakeUp();
void goToSleep();

#endif /* SLEEP_MODES_H_ */