/*
 * GEIGER.h
 *
 *  Created on: Apr 5, 2017
 *      Author: epiczero
 */

#ifndef GEIGER_H_
#define GEIGER_H_
#include "FreeRTOS.h"
#include "task.h"

//NOTES:
//the GEIGER counter devkit is 5V logic / do not connect directly to 3v3 STM32F4 mcu.
//USES EXT11 gpio ISR to increment the click count, triggered on rising edge from Geiger "out" pin.
//ALTERNATIVELY,UART3 RX can be used to process geiger clicks, uses same GEIG_incClick() as EXT11.
//USES tim6 timer ISR callback (in main.c) to unblock processGEIGER() task.

//public struct of most recent Geiger counter data
typedef struct GEIG_pub { //
   uint8_t CPS;
   uint32_t CPM;
   uint32_t timestamp;
} GEIG_pub;
GEIG_pub GEIGpub;//still needed?

void GEIG_init(); //Called before threads start;
void GEIG_incClick(); //Call from external interrupt handler on rising edge;
void GEIG_tmrHandler(); //Called in main.c tim6 handler @ 1Hz
void GEIG_processGEIGER(); // called in higher priority task, released by timer.
void getGEIGER(GEIG_pub* myGEIGER);
#endif /* GEIGER_H_ */
