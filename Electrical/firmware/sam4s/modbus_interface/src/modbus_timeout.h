#ifndef MODBUS_TIMEOUT_H
#define MODBUS_TIMEOUT_H

#include "modbus_interface.h"

/* Include this in projects that require timeout capabilities for modbus (for example, to cut control to motor if no comms received).
 * Projects that include this must also install the Timer Counter (TC) module in ASF Wizard.
 */

void TC0_Handler(void) {
	// If an overflow occurred, increase elapsed by 2000 ms
	// since this happens every 2 seconds
	// We read the register status to clear overflow flag
	if (tc_get_status(TC0, 0) & TC_SR_COVFS) {
		elapsed_ms += 2000;
	}
}

#endif
