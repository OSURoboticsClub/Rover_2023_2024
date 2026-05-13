/*
 * IncFile1.h
 *
 * Created: 2/3/2022 3:37:11 PM
 *  Author: Anthony
 */ 


#ifndef REGISTERS_H_
#define REGISTERS_H_

//slaveID
#define SLAVEID 1

//int Registers

//#define speed 0

//float Registers

#define CELL1_VOLTAGE			0
#define CELL2_VOLTAGE			1
#define CELL3_VOLTAGE			2
#define CELL4_VOLTAGE			3
#define CELL5_VOLTAGE			4
#define CELL6_VOLTAGE			5

#define BATT_TEMP1				6
#define BATT_TEMP2				7
#define BATT_TEMP3				8
#define BATT_TEMP4				9
#define BATT_TEMP5				10
#define BATT_TEMP6				11

#define BATT_CURRENT			12
#define PCB_SHUNT_TEMP			13
#define PCB_FET_TEMP			14

#define BATTERY_OUTPUT_VOLTAGE	15

//#define temperature 0

//char Registers

//#define roverState 0

//Bool Registers

#define OVER_TEMP_PROTEC		0
#define UNDER_TEMP_PROTEC		1
#define OVER_VOLT_PROTEC		2
#define UNDER_VOLT_PROTEC		3
#define OVER_CURRENT_PROTEC		4
#define CELL_IMBALANCE_PROTEC	5
#define UPS						6

#endif /* REGISTERS_H_ */