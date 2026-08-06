/*
 * main.h
 *
 * Created: 4/19/2022 3:55:36 PM
 *  Author: Anthony
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#define ADC_CLK_FREQ			250000
#define QTY_TRACKING_PERIODS	3
#define TRANSFER_PERIOD			2

#define QTY_ANALOG_SOURCES	16
#define CELLVIDXS			0
#define CELL1IDX			0
#define CELL2IDX			1
#define CELL3IDX			2
#define CELL4IDX			3
#define CELL5IDX			4
#define CELL6IDX			5
#define TEMPIDXS			6
#define TEMP1IDX			6
#define TEMP2IDX			7
#define TEMP3IDX			8
#define TEMP4IDX			9
#define TEMP5IDX			10
#define TEMP6IDX			11
#define CURRENTIDX			12
#define SHUNTTEMPIDX		13
#define FETTEMPIDX			14
#define STACKVOLTIDX		15

#define CELLV_CHANNEL		ADC_CHANNEL_0
#define CURRENT_CHANNEL		ADC_CHANNEL_1
#define STACK_CHANNEL		ADC_CHANNEL_2
#define EXTERN_TEMP_CHANNEL	ADC_CHANNEL_3
#define FET_TEMP_CHANNEL	ADC_CHANNEL_4
#define SHUNT_TEMP_CHANNEL	ADC_CHANNEL_5

#define QTY_PROTECTIONS		6
#define OVERTEMPIDX			0
#define UNDERTEMPIDX		1
#define OVERVOLTIDX			2
#define UNDERVOLTIDX		3
#define OVERCURRENTIDX		4
#define CELLIMBALANCEIDX	5

#define BLINK_TC			1
#define ADC_TC				0
#define WAKEUP_TC			2


#define OVER_VOLTAGE_THRESHOLD		4.4
#define UNDER_VOLTAGE_THRESHOLD		3.3
#define CELL_OVER_TEMP_THRESHOLD	60.0
#define PCB_OVER_TEMP_THRESHOLD		80.0
#define UNDER_TEMP_THRESHOLD		1.0
#define IST_OVER_CURRENT_THRESHOLD	500.0			//These numbers are temperarily high till the current sensor is working better.
#define CTS_OVER_CURRENT_THRESHOLD	500.0
#define CELL_IMBALANCE_THRESHOLD	0.4

#define SIZE_OF_DATA_FLASH		QTY_PROTECTIONS + (3 * QTY_ANALOG_SOURCES) + 3
#define FLASH_DATA_PAGE_ADDRESS (IFLASH0_ADDR + IFLASH0_SIZE - IFLASH0_PAGE_SIZE * 8)


void analogCalculate();
void adcSetup();
void pinWrite(Pio*, const uint32_t, bool);
void triggerProtections();
bool overTemp();
bool underTemp();
bool overVolt();
bool underVolt();
bool overCurrent();
bool cellImbalance();
void protec();
void printStatus();
void printInstructions();
void checkUSB();
void clearProtections();
void printString(char*);
void loadFromFlash();
static float convertToFloat(const uint32_t);
void updateRegisters();
void checkIfPeriodicWakeup();
static uint32_t convertToInt32(const float);

float MovingAverageADCData[QTY_ANALOG_SOURCES] = {4.0,4.0,4.0,4.0,4.0,4.0, 25.0,25.0,25.0,25.0,25.0,25.0, 10.0, 25.0,25.0, 24.0};
float ctsAverageCurrent = 0.0;
float minimumValues[QTY_ANALOG_SOURCES];
float maximumValues[QTY_ANALOG_SOURCES];
bool activeProtections[QTY_PROTECTIONS];
bool batteryStable = true, ignoreProtections = false;
bool protectionActive = false;

#endif /* MAIN_H_ */