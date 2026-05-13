/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

/* THESE ARE DEFINED HERE TO GET WARNINGS TO SHUTUP!
 * The ASF library generates warnings if they aren't defined, then sets them to default values.
 * So manually define them with those default values so it won't complain. */
#define BOARD_FREQ_SLCK_XTAL      (32768UL)
#define BOARD_FREQ_SLCK_BYPASS    (32768UL)
#define BOARD_FREQ_MAINCK_XTAL    (12000000UL)
#define BOARD_FREQ_MAINCK_BYPASS  (12000000UL)
#define BOARD_OSC_STARTUP_US      (15625UL)

#endif // CONF_BOARD_H
