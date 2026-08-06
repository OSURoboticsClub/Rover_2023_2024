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

#define RS485_NRE				PIO_PA12
#define RS485_NRE_PORT			PIOA
#define RS485_DE				PIO_PA13
#define RS485_DE_PORT			PIOA

#define TEMP_SEL0				PIO_PA8
#define TEMP_SEL0_PORT			PIOA
#define TEMP_SEL1				PIO_PA9
#define TEMP_SEL1_PORT			PIOA
#define TEMP_SEL2				PIO_PA10
#define TEMP_SEL2_PORT			PIOA

#define CELL_SEL0				PIO_PA4
#define CELL_SEL0_PORT			PIOA
#define CELL_SEL1				PIO_PA5
#define CELL_SEL1_PORT			PIOA
#define CELL_SEL2				PIO_PA6
#define CELL_SEL2_PORT			PIOA

#define AFE_EN					PIO_PA7
#define AFE_EN_PORT				PIOA

#define PWR_SW					PIO_PA0
#define PWR_SW_PORT				PIOA

#define NBAT_EN					PIO_PA11
#define NBAT_EN_PORT			PIOA

#define BOARD_LED				PIO_PA3
#define BOARD_LED_PORT			PIOA

#define USB_SNS					PIO_PA14
#define USB_SNS_PORT			PIOA

#define CELLV_SNS				PIO_PA17
#define CELLV_SNS_PORT			PIOA

#define CURRENT_SNS				PIO_PA18
#define CURRENT_SNS_PORT		PIOA

#define STACK_SNS				PIO_PA19
#define	STACK_SNS_PORT			PIOA

#define EXT_TEMP_SNS			PIO_PA20
#define EXT_TEMP_SNS_PORT		PIOA

#define FETTEMP_SNS				PIO_PB0
#define FETTEMP_SNS_PORT		PIOB

#define SHUNTTEMP_SNS			PIO_PB1
#define SHUNTTEMP_SNS_PORT		PIOB

#define PERIODIC_WAKEUP_TIME 15 //(in minutes)

#endif // CONF_BOARD_H