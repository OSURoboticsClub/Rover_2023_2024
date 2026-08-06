/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <math.h>
#include <string.h>
#include <main.h>
#include "modbus_uart1.h"
#include <registers.h>
#include <interupt_handlers.h>

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	
	sysclk_init();
	board_init();
	adcSetup();
	loadFromFlash();
	checkIfPeriodicWakeup();

	/* Insert application code here, after the board has been initialized. */
	
	while(true){
		modbus_update();
		analogCalculate();
		triggerProtections();
		checkUSB();
		updateRegisters();
	}
}


/****************************
*	ADC and data processing
*****************************/

int rawADCData[QTY_ANALOG_SOURCES];
float processedADCData[QTY_ANALOG_SOURCES];
bool needsProcessing[QTY_ANALOG_SOURCES];
int currentExternTempSensor = 0;
int currentExternCell		= 0;
float PCBTempLookUpTable[1024];
float ExtTempLookUpTable[1024];
int CellVcounter			= 0;

void adcSetup(){
	//Generate temperature lookup tables.
	float PCBBeta = 4700.0;
	float ExBeta = 3950.0;
	for(int i=0;i<1024;i++){
		float voltage = ((float)(i)/1023)*3.3;
		PCBTempLookUpTable[i] = (1.0/((log(voltage/(3.3-voltage))/PCBBeta)+(1.0/(25.0+273.15))))-273.15;
		ExtTempLookUpTable[i] = (1.0/((log(voltage/(3.3-voltage))/ExBeta)+(1.0/(25.0+273.15))))-273.15;
	}
	
	//enable and start adc.
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC,sysclk_get_peripheral_hz(),ADC_CLK_FREQ,ADC_STARTUP_TIME_4);
	adc_configure_timing(ADC, QTY_TRACKING_PERIODS, ADC_SETTLING_TIME_2, TRANSFER_PERIOD);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);		//Software Trigger, no freerun (I think this is defauilt)
	ADC->ADC_EMR |= ADC_EMR_TAG;					//appends channel tag to last converted data
	adc_enable_channel(ADC, CELLV_CHANNEL);
	adc_enable_channel(ADC, CURRENT_CHANNEL);
	adc_enable_channel(ADC, STACK_CHANNEL);
	adc_enable_channel(ADC, EXTERN_TEMP_CHANNEL);
	adc_enable_channel(ADC, FET_TEMP_CHANNEL);
	adc_enable_channel(ADC, SHUNT_TEMP_CHANNEL);
	adc_enable_interrupt(ADC,ADC_IER_DRDY);
	NVIC_EnableIRQ(ADC_IRQn);
	
	pmc_enable_periph_clk(ID_TC0);
	tc_init(TC0, ADC_TC,
			TC_CMR_WAVSEL_UP_RC
			|TC_CMR_WAVE
			| TC_CMR_TCCLKS_TIMER_CLOCK4);
	tc_write_rc(TC0,ADC_TC,187);				  //causes timer to reset every Sets everything to 5000Hz sampling (except cells are at 50Hz)
	tc_enable_interrupt(TC0, ADC_TC, TC_IER_CPCS);
	NVIC_EnableIRQ(TC0_IRQn);
	tc_start(TC0,ADC_TC);
}


void ADC_Handler(){
	uint32_t ADCState = ADC->ADC_ISR;
	if((ADCState & ADC_ISR_DRDY) == ADC_ISR_DRDY){
		int lastConvertedRaw = ADC->ADC_LCDR;
		int lastConvertedChannel = (lastConvertedRaw & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos;
		int lastConvertedData = lastConvertedRaw & ADC_LCDR_LDATA_Msk;
		if(lastConvertedData == 4095) return;											//throw out bad data
		
		switch(lastConvertedChannel){
		
			case CELLV_CHANNEL:
				rawADCData[CELLVIDXS + currentExternCell] = lastConvertedData;
				needsProcessing[CELLVIDXS + currentExternCell] = true;
				if(currentExternCell == 5) currentExternCell = 0;
				else currentExternCell++;
				pinWrite(CELL_SEL0_PORT,CELL_SEL0,(bool)(currentExternCell&1));
				pinWrite(CELL_SEL1_PORT,CELL_SEL1,(bool)(currentExternCell&2));
				pinWrite(CELL_SEL2_PORT,CELL_SEL2,(bool)(currentExternCell&4));
				adc_disable_channel(ADC,CELLV_CHANNEL);
				CellVcounter = 0;
				break;
			
			case CURRENT_CHANNEL:
				rawADCData[CURRENTIDX] = adc_get_channel_value(ADC,CURRENT_CHANNEL);
				needsProcessing[CURRENTIDX] = true;
				break;
			
			case STACK_CHANNEL:
				rawADCData[STACKVOLTIDX] = adc_get_channel_value(ADC,STACK_CHANNEL);
				needsProcessing[STACKVOLTIDX] = true;
				break;
			
			case EXTERN_TEMP_CHANNEL:
				rawADCData[TEMPIDXS + currentExternTempSensor] = adc_get_channel_value(ADC,EXTERN_TEMP_CHANNEL);
				needsProcessing[TEMPIDXS + currentExternTempSensor] = true;
				if(currentExternTempSensor == 5) currentExternTempSensor = 0;
				else currentExternTempSensor++;
				pinWrite(TEMP_SEL0_PORT,TEMP_SEL0,(bool)(currentExternTempSensor&1));
				pinWrite(TEMP_SEL1_PORT,TEMP_SEL1,(bool)(currentExternTempSensor&2));
				pinWrite(TEMP_SEL2_PORT,TEMP_SEL2,(bool)(currentExternTempSensor&4));
				break;
		
			case FET_TEMP_CHANNEL:
				rawADCData[FETTEMPIDX] = adc_get_channel_value(ADC,FET_TEMP_CHANNEL);
				needsProcessing[FETTEMPIDX] = true;
				break;
			
			case SHUNT_TEMP_CHANNEL:
				rawADCData[SHUNTTEMPIDX] = adc_get_channel_value(ADC,SHUNT_TEMP_CHANNEL);
				needsProcessing[SHUNTTEMPIDX] = true;
				break;
		};
	}
}

float cellMovingAverageCoef = 0.55;						//This number was chosen to have good 0.1s response for 50Hz processing.
float tempMovingAverageCoef = 0.005;					//This number was chosen to give good 0.1s response for 5000Hz sampling.
float ctsCurrentMovingAverageCoef = 0.0002;				//This number was chosen to give good 5s response for 5000Hz sampling. this will monitor cts current.
float istCurrentMovingAverageCoef =	0.55;				//This number was chosen to give good 0.001s response for 5000Hz sampling. this will monitor ist current
float stackMovingAverageCoef = 0.55;					//This number was chosen to give good 0.001s response for 5000Hz sampling.

void analogCalculate(){
	if( needsProcessing[CELL1IDX] || needsProcessing[CELL2IDX] || needsProcessing[CELL3IDX] || needsProcessing[CELL4IDX] || needsProcessing[CELL5IDX] || needsProcessing[CELL6IDX]){
		int processIDX;
		for(int i=CELL1IDX;i<=CELL6IDX;i++) if(needsProcessing[i]) processIDX = i;
		processedADCData[processIDX] = (3.3*(rawADCData[processIDX]/4095.0)) * (1.0/0.759);
		MovingAverageADCData[processIDX] = (processedADCData[processIDX]*cellMovingAverageCoef) + (MovingAverageADCData[processIDX]*(1.0-cellMovingAverageCoef));
		maximumValues[processIDX] = (MovingAverageADCData[processIDX] > maximumValues[processIDX]) ? MovingAverageADCData[processIDX] : maximumValues[processIDX];
		minimumValues[processIDX] = (MovingAverageADCData[processIDX] < minimumValues[processIDX]) ? MovingAverageADCData[processIDX] : minimumValues[processIDX];
		needsProcessing[processIDX] = false;
	}
	if(needsProcessing[TEMP1IDX] || needsProcessing[TEMP2IDX] || needsProcessing[TEMP3IDX] || needsProcessing[TEMP4IDX] || needsProcessing[TEMP5IDX] || needsProcessing[TEMP6IDX]){
		int processIDX;
		for(int i=TEMP1IDX;i<=TEMP6IDX;i++) if(needsProcessing[i]) processIDX = i;
		processedADCData[processIDX] = ExtTempLookUpTable[(int)(rawADCData[processIDX]/4)];
		MovingAverageADCData[processIDX] = processedADCData[processIDX]*tempMovingAverageCoef + MovingAverageADCData[processIDX]*(1.0 - tempMovingAverageCoef);
		maximumValues[processIDX] = (MovingAverageADCData[processIDX] > maximumValues[processIDX]) ? MovingAverageADCData[processIDX] : maximumValues[processIDX];
		minimumValues[processIDX] = (MovingAverageADCData[processIDX] < minimumValues[processIDX]) ? MovingAverageADCData[processIDX] : minimumValues[processIDX];
		needsProcessing[processIDX] = false;
	}
	if(needsProcessing[CURRENTIDX]){
		processedADCData[CURRENTIDX] = (3.3*(rawADCData[CURRENTIDX]/4095.0)) * (1000/20);
		MovingAverageADCData[CURRENTIDX] = processedADCData[CURRENTIDX]*istCurrentMovingAverageCoef + MovingAverageADCData[CURRENTIDX]*(1.0-istCurrentMovingAverageCoef);
		ctsAverageCurrent = processedADCData[CURRENTIDX]*ctsCurrentMovingAverageCoef + ctsAverageCurrent*(1.0-ctsCurrentMovingAverageCoef);
		maximumValues[CURRENTIDX] = (MovingAverageADCData[CURRENTIDX] > maximumValues[CURRENTIDX]) ? MovingAverageADCData[CURRENTIDX] : maximumValues[CURRENTIDX];
		minimumValues[CURRENTIDX] = (MovingAverageADCData[CURRENTIDX] < minimumValues[CURRENTIDX]) ? MovingAverageADCData[CURRENTIDX] : minimumValues[CURRENTIDX];
		needsProcessing[CURRENTIDX] = false;
	}
	if(needsProcessing[SHUNTTEMPIDX]){
		processedADCData[SHUNTTEMPIDX] = PCBTempLookUpTable[(int)(rawADCData[SHUNTTEMPIDX]/4)];
		MovingAverageADCData[SHUNTTEMPIDX] = processedADCData[SHUNTTEMPIDX]*tempMovingAverageCoef + MovingAverageADCData[SHUNTTEMPIDX]*(1.0-tempMovingAverageCoef);
		maximumValues[SHUNTTEMPIDX] = (MovingAverageADCData[SHUNTTEMPIDX] > maximumValues[SHUNTTEMPIDX]) ? MovingAverageADCData[SHUNTTEMPIDX] : maximumValues[SHUNTTEMPIDX];
		minimumValues[SHUNTTEMPIDX] = (MovingAverageADCData[SHUNTTEMPIDX] < minimumValues[SHUNTTEMPIDX]) ? MovingAverageADCData[SHUNTTEMPIDX] : minimumValues[SHUNTTEMPIDX];
		needsProcessing[SHUNTTEMPIDX] = false;
	}
	if(needsProcessing[FETTEMPIDX]){
		processedADCData[FETTEMPIDX] = PCBTempLookUpTable[(int)(rawADCData[FETTEMPIDX]/4)];
		MovingAverageADCData[FETTEMPIDX] = processedADCData[FETTEMPIDX]*tempMovingAverageCoef + MovingAverageADCData[FETTEMPIDX]*(1.0-tempMovingAverageCoef);
		maximumValues[FETTEMPIDX] = (MovingAverageADCData[FETTEMPIDX] > maximumValues[FETTEMPIDX]) ? MovingAverageADCData[FETTEMPIDX] : maximumValues[FETTEMPIDX];
		minimumValues[FETTEMPIDX] = (MovingAverageADCData[FETTEMPIDX] < minimumValues[FETTEMPIDX]) ? MovingAverageADCData[FETTEMPIDX] : minimumValues[FETTEMPIDX];
		needsProcessing[FETTEMPIDX] = false;
	}
	if(needsProcessing[STACKVOLTIDX]){
		processedADCData[STACKVOLTIDX] = (rawADCData[STACKVOLTIDX]/4095.0)*3.30*11;
		MovingAverageADCData[STACKVOLTIDX] = processedADCData[STACKVOLTIDX]*stackMovingAverageCoef + MovingAverageADCData[STACKVOLTIDX]*(1.0-stackMovingAverageCoef);
		maximumValues[STACKVOLTIDX] = (MovingAverageADCData[STACKVOLTIDX] > maximumValues[STACKVOLTIDX]) ? MovingAverageADCData[STACKVOLTIDX] : maximumValues[STACKVOLTIDX];
		minimumValues[STACKVOLTIDX] = (MovingAverageADCData[STACKVOLTIDX] < minimumValues[STACKVOLTIDX]) && !(PIOA->PIO_ODSR & NBAT_EN) ? MovingAverageADCData[STACKVOLTIDX] : minimumValues[STACKVOLTIDX];
		needsProcessing[STACKVOLTIDX] = false;
	}
}



/****************************
*	Protections
*****************************/

void triggerProtections(){
	if(overTemp())
		activeProtections[OVERTEMPIDX] = true;
	else if(ignoreProtections)
		activeProtections[OVERTEMPIDX] = false;
		
	if(underTemp())
		activeProtections[UNDERTEMPIDX] = true;
	else if(ignoreProtections)
		activeProtections[UNDERTEMPIDX] = false;
		
	if(overVolt())
		activeProtections[OVERVOLTIDX] = true;
	else if(ignoreProtections)
		activeProtections[OVERVOLTIDX] = false;
		
	if(underVolt())
		activeProtections[UNDERVOLTIDX] = true;
	else if(ignoreProtections)
		activeProtections[UNDERVOLTIDX] = false;
		
	if(overCurrent())
		activeProtections[OVERCURRENTIDX] = true;
	else if(ignoreProtections)
		activeProtections[OVERCURRENTIDX] = false;
		
	if(cellImbalance())
		activeProtections[CELLIMBALANCEIDX] = true;
	else if(ignoreProtections)
		activeProtections[CELLIMBALANCEIDX] = false;
		
	batteryStable = !(activeProtections[OVERTEMPIDX] || activeProtections[UNDERTEMPIDX] || activeProtections[OVERVOLTIDX] || activeProtections[UNDERVOLTIDX] || activeProtections[OVERCURRENTIDX] || activeProtections[CELLIMBALANCEIDX]);
	
	if(!batteryStable && !ignoreProtections && !protectionActive){
		protec();
		protectionActive = true;
	}else if(batteryStable){
		ignoreProtections = false;
		pio_clear(BOARD_LED_PORT,BOARD_LED);
		pio_clear(NBAT_EN_PORT,NBAT_EN);
	}
}

bool overTemp(){
	for(int i=TEMP1IDX;i<=TEMP6IDX;i++)
		if(MovingAverageADCData[i] >= CELL_OVER_TEMP_THRESHOLD)
			return true;
	if((MovingAverageADCData[SHUNTTEMPIDX] >= PCB_OVER_TEMP_THRESHOLD) || (MovingAverageADCData[FETTEMPIDX] >= PCB_OVER_TEMP_THRESHOLD))
		return true;
	return false;
}
bool underTemp(){
	for(int i=TEMP1IDX;i<=TEMP6IDX;i++)
		if(MovingAverageADCData[i] <= UNDER_TEMP_THRESHOLD)
			return true;
	return false;
}
bool overVolt(){
	for(int i=CELL1IDX;i<=CELL6IDX;i++)
		if(MovingAverageADCData[i] >= OVER_VOLTAGE_THRESHOLD)
			return true;
	return false;
}
bool underVolt(){
	for(int i=CELL1IDX;i<=CELL6IDX;i++)
		if(MovingAverageADCData[i] <= UNDER_VOLTAGE_THRESHOLD)
			return true;
	if(!(PIOA->PIO_ODSR & NBAT_EN) && (MovingAverageADCData[STACKVOLTIDX] <= 6*UNDER_VOLTAGE_THRESHOLD))		//checks the overall output stack voltage if the output is enabled
		return true;
	return false;
}
bool overCurrent(){
	if(MovingAverageADCData[CURRENTIDX] >= IST_OVER_CURRENT_THRESHOLD)
		return true;
	if(ctsAverageCurrent >= CTS_OVER_CURRENT_THRESHOLD)
		return true;
	return false;
}
bool cellImbalance(){
	float minCell = 5.0, maxCell = 0;
	for(int i=CELL1IDX;i<=CELL6IDX;i++){
		if(MovingAverageADCData[i] < minCell) minCell = MovingAverageADCData[i];
		if(MovingAverageADCData[i] > maxCell) maxCell = MovingAverageADCData[i];
	}
	if((maxCell - minCell) > CELL_IMBALANCE_THRESHOLD)
		return true;
	return false;
}

void protec(){
	
	pio_set(NBAT_EN_PORT, NBAT_EN);
	
	pmc_enable_periph_clk(ID_TC1);
	tc_init(TC0, BLINK_TC,
			TC_CMR_WAVSEL_UP_RC
			|TC_CMR_WAVE
			| TC_CMR_TCCLKS_TIMER_CLOCK5);
	tc_write_rc(TC0,BLINK_TC,16000);
	tc_enable_interrupt(TC0, BLINK_TC, TC_IER_CPCS);
	tc_start(TC0,BLINK_TC);
	NVIC_EnableIRQ(TC1_IRQn);
}



/****************************
*	USB COMMS
*****************************/

void checkUSB(){
	if(udi_cdc_get_nb_received_data() > 0){
		char inChar = udi_cdc_getc();
		if (inChar == 0) return;
		if(inChar == 's')
			printStatus();
		else if(inChar == 'c'){
			clearProtections();
			char * message = "\n\rAll protections will be ignored till the battery is stable again\n\r";
			printString(message);
		}else
			printInstructions();
	}
}

void printStatus(){
	
	char str[256] = "\n\r\n\rCurrent Active Protections:\n\r";
	printString(str);
	sprintf(str,"Over Temperature:	%i\n\r", (int)(activeProtections[OVERTEMPIDX]));
	printString(str);
	sprintf(str,"Under Temperature:	%i\n\r", (int)(activeProtections[UNDERTEMPIDX]));
	printString(str);
	sprintf(str, "Over Voltage:		%i\n\r", (int)(activeProtections[OVERVOLTIDX]));
	printString(str);
	sprintf(str, "Under Voltage:		%i\n\r", (int)(activeProtections[UNDERVOLTIDX]));
	printString(str);
	sprintf(str, "Over Current:		%i\n\r", (int)(activeProtections[OVERCURRENTIDX]));
	printString(str);
	sprintf(str, "Cell Imbalance:		%i\n\r", (int)(activeProtections[CELLIMBALANCEIDX]));
	printString(str);
	
	char strink[256] = "\n\rName, Current Value, Smallest lifetime value, largest lifetime value\n\r";
	printString(strink);
	for (int i=0;i<6;i++){
		sprintf(str, "Cell_%i_voltage,%f,%f,%f\n\r",i+1,MovingAverageADCData[CELLVIDXS+i],minimumValues[CELLVIDXS+i],maximumValues[CELLVIDXS+i]);
		printString(str);
	}
	for (int i=0;i<6;i++){
		sprintf(str, "Ext_Temp_%i,%f,%f,%f\n\r",i+1,MovingAverageADCData[TEMPIDXS+i],minimumValues[TEMPIDXS+i],maximumValues[TEMPIDXS+i]);
		printString(str);
	}
	sprintf(str, "Current,%f,%f,%f\n\r",MovingAverageADCData[CURRENTIDX],minimumValues[CURRENTIDX],maximumValues[CURRENTIDX]);
	printString(str);
	sprintf(str, "Shunt_Temp,%f,%f,%f\n\r",MovingAverageADCData[SHUNTTEMPIDX],minimumValues[SHUNTTEMPIDX],maximumValues[SHUNTTEMPIDX]);
	printString(str);
	sprintf(str, "Fet_Temp,%f,%f,%f\n\r",MovingAverageADCData[FETTEMPIDX],minimumValues[FETTEMPIDX],maximumValues[FETTEMPIDX]);
	printString(str);
	sprintf(str, "Output_Voltage,%f,%f,%f\n\r",MovingAverageADCData[STACKVOLTIDX],minimumValues[STACKVOLTIDX],maximumValues[STACKVOLTIDX]);
	printString(str);
}

void printInstructions(){
	char * instructions = "\n\rPlease Type\"s\" to get the current status of the battery\n\rAfter determining the best action type \"c\" to temporarily clear all protections\n\r[WARNING] when the protections are cleared, the BMS will ignore all protections till the battery is stable once again\n\r This is indicated by the red LED turning off.\n\r";
	printString(instructions);
}

void clearProtections(){
	ignoreProtections = true;
	pio_clear(NBAT_EN_PORT, NBAT_EN);
	tc_stop(TC0,BLINK_TC);
	pio_set(BOARD_LED_PORT,BOARD_LED);
}




/****************************
*	Sleep
*****************************/

uint32_t flashPageBuffer[SIZE_OF_DATA_FLASH];
void goToSleep(){
	//turn off everything
	pio_set(NBAT_EN_PORT,NBAT_EN);
	pio_set(RS485_NRE_PORT,RS485_NRE);
	pio_clear(RS485_DE_PORT,RS485_DE);
	pio_clear(AFE_EN_PORT,AFE_EN);
	pio_clear(BOARD_LED_PORT,BOARD_LED);
	pio_clear(TEMP_SEL0_PORT,TEMP_SEL0);
	pio_clear(TEMP_SEL1_PORT,TEMP_SEL1);
	pio_clear(TEMP_SEL2_PORT,TEMP_SEL2);
	pio_clear(CELL_SEL0_PORT,CELL_SEL0);
	pio_clear(CELL_SEL1_PORT,CELL_SEL1);
	pio_clear(CELL_SEL2_PORT,CELL_SEL2);
	//pmc_disable_all_periph_clk();
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_DisableIRQ(TC1_IRQn);
	NVIC_DisableIRQ(TC2_IRQn);
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_DisableIRQ(ADC_IRQn);
	
	
	//Store to flash
	int buffIDX = 0;
	int i=0;
	for(int i=0; i < QTY_PROTECTIONS; i++){
		flashPageBuffer[i] = !activeProtections[i-buffIDX];
	}
	buffIDX = QTY_PROTECTIONS;
	
	flashPageBuffer[buffIDX++] = !batteryStable;
	
	flashPageBuffer[buffIDX++] = !ignoreProtections;
	
	flashPageBuffer[buffIDX++] = !protectionActive;
	
	for (int i=0;i<QTY_ANALOG_SOURCES;i++){
		flashPageBuffer[i+buffIDX] = convertToInt32(MovingAverageADCData[i]);
		flashPageBuffer[i+buffIDX + QTY_ANALOG_SOURCES] = convertToInt32(minimumValues[i]);
		flashPageBuffer[i+buffIDX + (QTY_ANALOG_SOURCES * 2)] = convertToInt32(maximumValues[i]);
	}
	
	uint32_t flashOK = flash_unlock(FLASH_DATA_PAGE_ADDRESS, SIZE_OF_DATA_FLASH * sizeof(uint32_t),0,0);
	if(flashOK != FLASH_RC_OK) return;
	flashOK = flash_erase_sector(FLASH_DATA_PAGE_ADDRESS);
	if(flashOK != FLASH_RC_OK) return;
	
	for(int i=0; i< 4;i++){
		uint32_t addr = FLASH_DATA_PAGE_ADDRESS + (i*0x40);
		flashOK = flash_write(addr, (&flashPageBuffer[i*16]), 16 * sizeof(uint32_t), 0);
		if(flashOK != FLASH_RC_OK) return;
	}
	
	//reset RTT for periodic wakeups
	RTT->RTT_MR |= RTT_MR_RTTRST;
	
	//enter sleep mode
	pmc_enable_periph_clk(ID_SUPC);
	supc_set_wakeup_mode(SUPC,SUPC_WUMR_RTTEN_ENABLE | SUPC_WUMR_WKUPDBC_512_SCLK);
	supc_set_wakeup_inputs(SUPC,SUPC_WUIR_WKUPEN0 | SUPC_WUIR_WKUPEN8 , SUPC_WUIR_WKUPT0 | SUPC_WUIR_WKUPT8);
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
	supc_enable_backup_mode(SUPC);
}





/****************************
*	MISC
*****************************/
void pinWrite(Pio* port, const uint32_t pin_mask, bool logic){
	if(logic) pio_set(port,pin_mask);
	else pio_clear(port,pin_mask);
}


void TC0_Handler(){
	if( (tc_get_status(TC0,ADC_TC) & TC_SR_CPCS) == TC_SR_CPCS)
		CellVcounter++;
		if(CellVcounter > 100){
			adc_enable_channel(ADC,CELLV_CHANNEL);
			CellVcounter = 0;
		}
		adc_start(ADC);
}

void TC1_Handler(){
	if( (tc_get_status(TC0,BLINK_TC) & TC_SR_CPCS) == TC_SR_CPCS)
		pio_toggle_pin_group(BOARD_LED_PORT,BOARD_LED);
}

void TC2_Handler(){
	if( (tc_get_status(TC0,WAKEUP_TC) & TC_SR_CPCS) == TC_SR_CPCS){
		goToSleep();
		pio_set(BOARD_LED_PORT,BOARD_LED);
	}
}


void RTT_Handler(){
	if((rtt_get_status(RTT) & RTT_SR_ALMS) == RTT_SR_ALMS){
		RTT->RTT_MR |= RTT_MR_RTTRST;
	}
}

void printString(char* str){
	for(int i=0;i<strlen(str);i++){
		while(!udi_cdc_is_tx_ready()){}					//this is quick and easy but can be blocking for very long messages. Normally OK.
		udi_cdc_putc(str[i]);
	}
}

void loadFromFlash(){
	uint32_t flashOk = flash_init(FLASH_ACCESS_MODE_128, 6);
	if(flashOk != FLASH_RC_OK) return;
	flashOk = flash_unlock(FLASH_DATA_PAGE_ADDRESS, FLASH_DATA_PAGE_ADDRESS + (SIZE_OF_DATA_FLASH)*4,0,0);
	if(flashOk != FLASH_RC_OK) return;
	uint32_t *pageContents = (uint32_t *) FLASH_DATA_PAGE_ADDRESS;
	
	int flashIdx = 0;
	
	for(int i = 0;i < QTY_PROTECTIONS; i++){
		activeProtections[i-flashIdx] = !(bool)(pageContents[i]);
	}
	flashIdx = QTY_PROTECTIONS;
	
	batteryStable = !(bool)(pageContents[flashIdx++]);
	
	ignoreProtections = !(bool)(pageContents[flashIdx++]);
	
	protectionActive = false;
	flashIdx++;
	
	for(int i=0;i<QTY_ANALOG_SOURCES;i++){
		float fromMemory = convertToFloat(pageContents[i+flashIdx]);
		MovingAverageADCData[i] = fromMemory == 0? MovingAverageADCData[i] : fromMemory;
		fromMemory = convertToFloat(pageContents[ i + flashIdx + QTY_ANALOG_SOURCES ]);
		minimumValues[i] = fromMemory == 0? MovingAverageADCData[i] : fromMemory;
		fromMemory = convertToFloat(pageContents[ i + flashIdx + (QTY_ANALOG_SOURCES * 2) ]);
		maximumValues[i] = fromMemory == 0? MovingAverageADCData[i] : fromMemory;
	}
	
	if(ignoreProtections){
		pio_set(BOARD_LED_PORT,BOARD_LED);
	}
}


static float convertToFloat(const uint32_t indata){
	union {
		uint32_t data;
		float data_f;
	} u;

	u.data = indata;
	if(u.data == 0xffffffff) return 0;
	return u.data_f;
}

static uint32_t convertToInt32(const float indata){
	union {
		uint32_t data;
		float data_f;
	} u;

	u.data_f = indata;
	return u.data;
}

void updateRegisters(){
	floatRegisters[CELL1_VOLTAGE] = MovingAverageADCData[CELL1IDX];
	floatRegisters[CELL2_VOLTAGE] = MovingAverageADCData[CELL2IDX];
	floatRegisters[CELL3_VOLTAGE] = MovingAverageADCData[CELL3IDX];
	floatRegisters[CELL4_VOLTAGE] = MovingAverageADCData[CELL4IDX];
	floatRegisters[CELL5_VOLTAGE] = MovingAverageADCData[CELL5IDX];
	floatRegisters[CELL6_VOLTAGE] = MovingAverageADCData[CELL6IDX];
	
	floatRegisters[BATT_TEMP1] = MovingAverageADCData[TEMP1IDX];
	floatRegisters[BATT_TEMP2] = MovingAverageADCData[TEMP2IDX];
	floatRegisters[BATT_TEMP3] = MovingAverageADCData[TEMP3IDX];
	floatRegisters[BATT_TEMP4] = MovingAverageADCData[TEMP4IDX];
	floatRegisters[BATT_TEMP5] = MovingAverageADCData[TEMP5IDX];
	floatRegisters[BATT_TEMP6] = MovingAverageADCData[TEMP6IDX];
	
	floatRegisters[BATT_CURRENT] = MovingAverageADCData[CURRENTIDX];
	floatRegisters[PCB_SHUNT_TEMP] = MovingAverageADCData[SHUNTTEMPIDX];
	floatRegisters[PCB_FET_TEMP] = MovingAverageADCData[FETTEMPIDX];
	floatRegisters[BATTERY_OUTPUT_VOLTAGE] = MovingAverageADCData[STACKVOLTIDX];
	
	boolRegisters[OVER_TEMP_PROTEC] = activeProtections[OVERTEMPIDX];
	boolRegisters[UNDER_TEMP_PROTEC] = activeProtections[UNDERTEMPIDX];
	boolRegisters[OVER_VOLT_PROTEC] = activeProtections[OVERVOLTIDX];
	boolRegisters[UNDER_VOLT_PROTEC] = activeProtections[UNDERVOLTIDX];
	boolRegisters[OVER_CURRENT_PROTEC] = activeProtections[OVERCURRENTIDX];
	boolRegisters[CELL_IMBALANCE_PROTEC] = activeProtections[CELLIMBALANCEIDX];
}

void checkIfPeriodicWakeup(){
	bool USBSense = pio_get(USB_SNS_PORT,PIO_TYPE_PIO_INPUT,USB_SNS);
	bool PWRSwitchSense = pio_get(PWR_SW_PORT,PIO_TYPE_PIO_INPUT,PWR_SW);
	if(PWRSwitchSense || USBSense){
		pio_get_interrupt_status(PIOA);
		NVIC_EnableIRQ(PIOA_IRQn);
		return;
	}else{
		pmc_enable_periph_clk(ID_TC2);
		tc_init(TC0, WAKEUP_TC,
			TC_CMR_WAVSEL_UP_RC
			|TC_CMR_WAVE
			| TC_CMR_TCCLKS_TIMER_CLOCK4);
		tc_write_rc(TC0,WAKEUP_TC,9375);				  //sets wakeup to be 0.1s
		tc_enable_interrupt(TC0, WAKEUP_TC, TC_IER_CPCS);
		NVIC_EnableIRQ(TC2_IRQn);
		tc_start(TC0,WAKEUP_TC);
	}
}