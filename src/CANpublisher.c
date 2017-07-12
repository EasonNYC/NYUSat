/*
 * CANpublisher.c
 *
 *  Created on: Apr 15, 2017
 *      Author: epiczero
 */


#include "CANpublisher.h"
#include "can.h"

void testCAN(void){
	uint8_t id =5;
	uint32_t timestamp = 6;
	uint32_t  data = 7;
	publish2CAN(id, timestamp, data);
}
