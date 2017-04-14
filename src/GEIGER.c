/*
 * GEIGER.c
 *
 *  Created on: Apr 11, 2017
 *      Author: epiczero
 */
#include "GEIGER.h"
#include "circarray.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define NUM_CPS_READINGS 60

//flags
static volatile uint8_t GEIG_CPSrdy = 0;
static volatile uint8_t GEIG_CPMrdy = 0;

//temporary variables
static volatile uint16_t GEIGERcnt = 0;
static volatile uint8_t GEIG_tmpCPS = 0;
static volatile uint8_t GEIG_tmpCPM = 0;
static uint32_t GEIG_timestamp = 0;

//Circular Array
static volatile CircArr_InitTypeDef GEIG_carr;

extern volatile uint32_t msTicks;

//FreeRTOS related
static volatile TaskHandle_t GEIG_xTaskHandle = NULL;   //used to notify main Geiger process task its time to calculate CPS and CPM
static volatile SemaphoreHandle_t mutex_GEIGERDATA; 		//used for public R/W access to most recent CPM and CPM

//initialize the Geiger counter peripheral. Call before FreeRTOS threads start.
void GEIG_init() {

	GEIG_xTaskHandle = xTaskGetHandle("GEIGERTask");
	configASSERT(GEIG_xTaskHandle);
	mutex_GEIGERDATA = xSemaphoreCreateMutex();

	initCircArray(&GEIG_carr,NUM_CPS_READINGS);

	GEIG_CPSrdy = 1; //CPS data ready
	GEIG_CPMrdy = 0; //CPM data explicitly not yet ready
}

//Adds incoming geiger clicks to a temporary CPS variable that resets every second.
//This function is called inside the EXT11 pin ISR handler on rising edge or (alternatively) via Usart3 RX pin handler.
void GEIG_incClick() {

	configASSERT(GEIGERcnt < 256);
	GEIGERcnt++;
}

//sum the last 60 CPS readings and return total.
void GEIG_calcCPM(uint8_t* cpm) {
	configASSERT(cpm != NULL);

	uint32_t total = 0;
	for(uint8_t i = 0; i < NUM_CPS_READINGS; i++){
		total += GEIG_carr.buf[i];
	}

	*cpm = total;

}

//stores current CPS to temporary variable. Overwrites oldest reading in the circ array.
void GEIG_logCPS() {
	if(!GEIG_CPSrdy) {
		return;
	}

	//save timestamp
	GEIG_timestamp = msTicks;

	//temporary variable for transfer to public struct.
	GEIG_tmpCPS = GEIGERcnt;

	//Store CPS reading in array. flag ready to start collecting CPM.
	if(buf_putbyte(&GEIG_carr, GEIGERcnt) == 0){
		GEIG_CPMrdy = 1;
	}
}

//Calc CPM and it in temporary variable.
void GEIG_logCPM() {
	if(!GEIG_CPMrdy) {
		return;
	}

	//calculate and store CPM
	GEIG_calcCPM(&GEIG_tmpCPM);
}

//resets the current Geiger click count. Called in geiger process thread
void GEIG_resetClicks(){
	GEIGERcnt = 0;
}

//Called in tim6 timer ISR handler. Unblocks Geiger process thread @ 1Hz.
void GEIG_tmrHandler(){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(GEIG_xTaskHandle == NULL) {
		return; //geiger not initialized.
	}
	//configASSERT( GEIG_xTaskHandle != NULL );
	vTaskNotifyGiveFromISR( GEIG_xTaskHandle, &xHigherPriorityTaskWoken );//unblock geiger process thread

	//force a likely context switch to GEIGER process thread
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

//creates struct of most recent good data (@1Hz). non-blocking w/ timeout
uint8_t GEIG_cpyDATA() {
	uint32_t ret = 0;

	//copy variables to struct
	if ( xSemaphoreTake( mutex_GEIGERDATA, (BaseType_t)10) == pdTRUE){
	GEIGpub.timestamp = GEIG_timestamp;
	GEIGpub.CPS = GEIG_tmpCPS;
	GEIGpub.CPM = GEIG_tmpCPM;
	ret = 1; //copy succeeded
	xSemaphoreGive(mutex_GEIGERDATA);
	}

	return ret;
}


//main Geiger process thread, called in a loop in a freeRTOS task, blocks until tim6 1Hz timer
//ISR handler unblocks it each second.
void GEIG_processGEIGER(){

	//const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );


	//*** block! ***
	const TickType_t xMaxBlockTime = portMAX_DELAY;
	uint32_t notifier = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );
	//Block is released every 1Hz by GEIG_timrHandler() (called in tim6 irq handler in main.c)
	//note: notifier gets 1 from VTaskNotifyGive in ISR. pdTrue clears that back to 0
	//notifier is just the semaphore counter value (implemented as a binary)
	//*** unblock ***



	//check notifier
	if(notifier) {
		GEIG_logCPS();
		GEIG_resetClicks();
		GEIG_logCPM();

		//keep trying to copy data until resources are free
		while ( !GEIG_cpyDATA() ){}

		//flag datardy to be published to CAN bus here?

	} else
	{
		//ERROR: timeout occured
		//Yeild?

		while(1){} //loop for now
	}
}



//Public function to get most recent data from geiger module. Blocking.
//todo: Pass timeout as argument in these public functions in all modules
void getGEIGER(GEIG_pub* myGEIGER) {
	if ( xSemaphoreTake( mutex_GEIGERDATA, portMAX_DELAY) == pdTRUE){
		myGEIGER->timestamp = GEIGpub.timestamp;
		myGEIGER->CPS = GEIGpub.CPS;
		myGEIGER->CPM = GEIGpub.CPM;
	xSemaphoreGive(mutex_GEIGERDATA);}

}



