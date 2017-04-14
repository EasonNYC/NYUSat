/*
 * GPS.h
 *
 *  Created on: Mar 4, 2017
 *      Author: epiczero
 */

#ifndef GPS_GPS_H_
#define GPS_GPS_H_

//public struct for sending GPS data to other modules
typedef struct GPS_pub {
   uint32_t timestamp;
   uint32_t pps_timestamp;
   uint8_t fixmode;
   uint8_t numsv;
   uint32_t gnss_time_of_week;
   float latitude;
   float longitude;
   float altitude_sealvl; //height above sealevel
} GPS_pub;
GPS_pub GPSpub;

//public struct to monitor blips and bloops for debug
typedef struct GPS_status {
	uint32_t chars_recvd;
   uint32_t s1_error; //start byte 1 not expected byte. some errors here ok (reason: cycling thru unhandled message ID's and msgs)
   uint32_t s2_error; //start byte 2.fewer errors here ok. Same reason, sometimes first start byte coincidentally in other msg payloads)
   uint32_t payload_length_error;
   uint32_t payloadOK; //should incr at 1HZ
   uint32_t chksum_error; //
   uint32_t t1_error;//trailer byte1 not expected byte
   uint32_t t2_error; //
   uint32_t numNAVmsgs;
   int32_t msTicks_error; //curr msticks - predicted using 1pps
} GPS_status;
GPS_status GPSstatus;

typedef struct GPS_time {
	uint8_t DayOfWeek;
	uint8_t Hour;
	uint8_t minute;
	uint8_t second;
	uint32_t mcu_timestamp; //via msTicks
} GPS_time;
GPS_time GPStime;

uint8_t GPS_available(void);
uint8_t GPS_getByte(void);
void GPS_putByte(uint8_t rxbyte);
void GPS_init(void);
void GPS_debug(char* s, SemaphoreHandle_t* m);
void processGPS(void);
float getLatitude(void);
float getLongitude(void);
uint8_t getNumSats(void);
float getAltitudeSealvl(void);
uint8_t getFix(void);
void GPS_ppsHandler(uint32_t cur_msTicks);
uint8_t getGPS(GPS_pub* myGPS); //for public mass consumption
#endif /* GPS_GPS_H_ */
