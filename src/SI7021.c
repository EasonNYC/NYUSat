/*
 * SI7021.c
 *
 *  Created on: Apr 4, 2017
 *      Author: epiczero
 */

#include "SI7021.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define DEVICESI 0x40
const uint8_t addr = DEVICESI<<1;//si7021 i2c address left aligned for sending
const uint8_t SIREG_RH = 0xE5; //RH in HOLD MODE, 0xF5 RH no hold
const uint8_t SIREG_TEMP = 0xE0; //get Temperature from previous RH measurement

volatile uint8_t RHdata[2] = {0,0};
volatile uint8_t TEMPdata[2] = {0,0};


static volatile SI7021_pub tmpSI7021data;

static SemaphoreHandle_t mutex_SI7021DATA = NULL;

void SI7021_init(void){
	//init read/write mutex
	mutex_SI7021DATA = xSemaphoreCreateMutex();

}

void processSI7021(void){

		//getRH
	    I2C_get16(addr,SIREG_RH, &RHdata);

	    //calculate Relative humidity (as a percentage)
	    uint16_t RHcode = (uint16_t)RHdata[0] << 8 | (uint16_t)RHdata[1];

	    //get TempC from previous RH measurement
	    I2C_get16(addr,SIREG_TEMP, &TEMPdata);

	    //calc tempC
	    uint16_t tempcode = (uint16_t)TEMPdata[0] << 8 | (uint16_t)TEMPdata[1];

	    //copy data using mutex
	    if ( xSemaphoreTake( mutex_SI7021DATA, portMAX_DELAY) == pdTRUE){
	    	tmpSI7021data.RelativeHumidity_PCT = (125.0*(float)(RHcode)/65536.0) - 6.0;
	    	tmpSI7021data.Temperature_DegC = (175.72*(float)(tempcode)/65536.0) - 46.85;
	    	SI7021status.readings++;
	    	xSemaphoreGive(mutex_SI7021DATA);
	    }

}

//for public reading of recent humidity and temperature data
void getSI7021(SI7021_pub* mySI7021){
	if ( xSemaphoreTake( mutex_SI7021DATA, portMAX_DELAY) == pdTRUE){//use mutex
		*mySI7021 = tmpSI7021data;
		xSemaphoreGive(mutex_SI7021DATA);
	}
}
