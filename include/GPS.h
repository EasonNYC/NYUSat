/*
 * GPS.h
 *
 *  Created on: Mar 4, 2017
 *      Author: epiczero
 */

#ifndef GPS_GPS_H_
#define GPS_GPS_H_

//public struct to toss around GPS data to other modules
typedef struct GPS_pub { //payload of 59 bytes
   uint8_t fixmode;
   uint8_t numsv;
   uint32_t gnss_time_of_week; //
   float latitude; //in decdeg mult by 10^7?
   float longitude;
   float altitude_sealvl; //height above sealevel
} GPS_pub;
GPS_pub GPSpub;

//public struct to monitor blips and bloops for debug
typedef struct GPS_status {
   uint32_t s1_error; //start byte 1 not expected byte. some errors here ok (reason: cycling thru unhandled message ID's and msgs)
   uint32_t s2_error; //start byte 2.fewer errors here ok. Same reason, sometimes first start byte coincidentally in other msg payloads)
   uint32_t payload_length_error;
   uint32_t payloadOK; //should incr at 1HZ
   uint32_t chksum_error; //
   uint32_t t1_error;//trailer byte1 not expected byte
   uint32_t t2_error; //
   uint32_t numNAVmsgs;
} GPS_status;
GPS_status GPSstatus;


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
uint8_t getGPS(GPS_pub* myGPS); //for public mass consumption
#endif /* GPS_GPS_H_ */
