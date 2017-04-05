/*
 * SI7021.h
 *
 *  Created on: Apr 4, 2017
 *      Author: epiczero
 */

#ifndef SI7021_H_
#define SI7021_H_

//public struct to pass around temperature and humidity to other modules
typedef struct SI7021_pub {
   float Temperature_DegC;
   float RelativeHumidity_PCT;
} SI7021_pub;
SI7021_pub SI7021pub;


//public struct to monitor device health
typedef struct SI7021_status {
   long readings;
} SI7021_status;
SI7021_status SI7021status;

void SI7021_init(void);
void processSI7021(void);
void getSI7021(SI7021_pub* mySI7021);

#endif /* SI7021_H_ */
