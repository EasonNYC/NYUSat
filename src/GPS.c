/*
 * GPS.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Eason Smith
 *
 *      The GPS class processes incoming binary GPS messages received via USART1
 */


#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "circarray.h"
#include "task.h"
#include "GPS.h"

#define START1 0xa0
#define START2 0xa1
#define TRAILER1 0x0d
#define TRAILER2 0x0a

#define MAXPAYLOADLEN 200

#define NAVID 0xa8
#define NAVLEN 59

//circular array
static volatile CircArr_InitTypeDef circarr; //first create 1 circular array buffer called msg

static uint8_t FLAG_OKTOCOPYNAVDATA = 0;
static uint32_t tmp_timestamp = 0;
static uint32_t tmp_PPStimestamp = 0;

static SemaphoreHandle_t mutex_CIRCARR_RW;
static SemaphoreHandle_t mutex_NAVDATA = NULL;

extern volatile uint32_t msTicks;

typedef union revint {
	int32_t myInt;
	uint8_t rawbytes[4];
	float myFloat;
} revint;

typedef struct NAVstruct { //payload of 59 bytes
   uint8_t id; //message type
   uint8_t fixmode;
   uint8_t numsv;
   uint16_t gnss_week_num;
   uint32_t gnss_time_of_week; //
   int32_t latitude; //in decdeg mult by 10^7?
   int32_t longitude;
   uint32_t altitude_ellip;
   uint32_t altitude_sealvl; //height above sealevel
   uint16_t gdop;
   uint16_t pdop;
   uint16_t hdop;
   uint16_t vdop;
   uint16_t tdop;
   int32_t ecef_x;
   int32_t ecef_y;
   int32_t ecef_z;
   int32_t ecef_velocity_x; //speed
   int32_t ecef_velocity_y;
   int32_t ecef_velocity_z;
   uint32_t mcu_timestamp;
   uint32_t pps_timestamp;
} NAVstruct;

/*todo: make byte aligned and try casting to struct without id
typedef union GPS {
	uint8_t raw[NAVLEN];
	NAVstruct current;
} GPS;
static GPS NAVdata, tmpNAVdata;  //can access NAVdata.current.latitude etc.
*/

NAVstruct NAVdata, tmpNAVdata;//use for now


typedef struct GPSmsg {//todo: add mcu timestamp or offset
   uint16_t payload_length;
   uint8_t payload[MAXPAYLOADLEN];
   uint8_t calculated_checksum;
   uint8_t received_checksum;
} GPSmsg;
static GPSmsg gpsmsg;


//GPS State Machine
typedef enum {GETSTART, GETSTART2, GETLENGTH, GETPAYLOAD, GETCHECKSUM, GETTRAILER, VALIDATE} GPS_STATE;
static GPS_STATE state = GETSTART;

//todo: wait on fix. wait on pps up. init time conversion between micro and sat
void GPS_init(void) {
	mutex_NAVDATA = xSemaphoreCreateMutex();
	mutex_CIRCARR_RW = xSemaphoreCreateMutex();

	initCircArray(&circarr,MAXPAYLOADLEN);//store incoming bytes
}

//called in StateMachine to process payload ID's of accepted payloads.
void HandlePayload(uint8_t* plmsg, uint8_t msglen){
uint8_t IDBYTE = plmsg[0]; //get ID

//GPS tmpNAVdata;
switch(IDBYTE){
	case NAVID:
		//transfer bytes  little endian
		tmpNAVdata.id = plmsg[0];
		tmpNAVdata.fixmode = plmsg[1];
		tmpNAVdata.numsv = plmsg[2];
		tmpNAVdata.gnss_week_num = (uint16_t)plmsg[3] << 8 | plmsg[4];
		tmpNAVdata.gnss_time_of_week = (uint32_t)plmsg[5] << 24 | (uint32_t)plmsg[6] << 16 | (uint32_t)plmsg[7] << 8 | plmsg[8];
		tmpNAVdata.latitude = (uint32_t)plmsg[9] << 24 | (uint32_t)plmsg[10] << 16 | (uint32_t)plmsg[11] << 8 | plmsg[12];
		tmpNAVdata.longitude = (uint32_t)plmsg[13] << 24 | (uint32_t)plmsg[14] << 16 | (uint32_t)plmsg[15] << 8 | plmsg[16];
		tmpNAVdata.altitude_ellip = (uint32_t)plmsg[17] << 24 | (uint32_t)plmsg[18] << 16 | (uint32_t)plmsg[19] << 8 | plmsg[20];
		tmpNAVdata.altitude_sealvl = (uint32_t)plmsg[21] << 24 | (uint32_t)plmsg[22] << 16 | (uint32_t)plmsg[23] << 8 | plmsg[24];
		tmpNAVdata.gdop = (uint16_t)plmsg[25] << 8 | plmsg[26];
		tmpNAVdata.pdop = (uint16_t)plmsg[27] << 8 | plmsg[28];
		tmpNAVdata.hdop = (uint16_t)plmsg[29] << 8 | plmsg[30];
		tmpNAVdata.vdop = (uint16_t)plmsg[31] << 8 | plmsg[32];
		tmpNAVdata.tdop = (uint16_t)plmsg[33] << 8 | plmsg[34];
		tmpNAVdata.ecef_x = (uint32_t)plmsg[35] << 24 | (uint32_t)plmsg[36] << 16 | (uint32_t)plmsg[37] << 8 | plmsg[38];
		tmpNAVdata.ecef_y = (uint32_t)plmsg[39] << 24 | (uint32_t)plmsg[40] << 16 | (uint32_t)plmsg[41] << 8 | plmsg[42];
		tmpNAVdata.ecef_z = (uint32_t)plmsg[43] << 24 | (uint32_t)plmsg[44] << 16 | (uint32_t)plmsg[45] << 8 | plmsg[46];
		tmpNAVdata.ecef_velocity_x = (uint32_t)plmsg[47] << 24 | (uint32_t)plmsg[48] << 16 | (uint32_t)plmsg[49] << 8 | plmsg[50];
		tmpNAVdata.ecef_velocity_y = (uint32_t)plmsg[51] << 24 | (uint32_t)plmsg[52] << 16 | (uint32_t)plmsg[53] << 8 | plmsg[54];
		tmpNAVdata.ecef_velocity_z = (uint32_t)plmsg[55] << 24 | (uint32_t)plmsg[56] << 16 | (uint32_t)plmsg[57] << 8 | plmsg[58];
		tmpNAVdata.mcu_timestamp = tmp_timestamp;
		tmpNAVdata.pps_timestamp = tmp_PPStimestamp;
		FLAG_OKTOCOPYNAVDATA = 1;
		break;
	default:
		break;// skip other message IDs for now...

	}
}

uint8_t GPS_available(void) {
	uint8_t b = 0;
	taskENTER_CRITICAL();
	b = buf_available(&circarr);
	taskEXIT_CRITICAL();
	return b;
}

//current callback function which puts an incoming uart byte into the GPS buffer.
void GPS_putByte(uint8_t rxbyte){
	buf_putbyte(&circarr,rxbyte);
}

//returns the next byte from the GPS buffer
uint8_t GPS_getByte(void){
	uint8_t b = 0;
	taskENTER_CRITICAL();
		buf_getbyte(&circarr,&b);
	taskEXIT_CRITICAL();
	return b;
}


//Main GPS State Machine processes incoming USART1 bytes into valid GPS Nav messages
void processGPS(void){
	switch(state){
	case GETSTART:
		//wait for 2 bytes over usart //split this up
		if(GPS_available()){

			    //save timestamp
				tmp_timestamp = msTicks;
				uint8_t received1 = GPS_getByte();

				if(received1 == START1){
					state = GETSTART2;
				}
				else
				{
					//(keep state the same)
					GPSstatus.s1_error++;
					break;
				}

    	}
		break;
	case GETSTART2:

		if(GPS_available()){
			uint8_t received2 = GPS_getByte();
			if(received2 == START2){
				state = GETLENGTH;
			}
			else {
				GPSstatus.s2_error++;
				state = GETSTART;
				break;
			}
		}
		break;
	case GETLENGTH:
		//wait for 2 bytes
		if(GPS_available() > 1){

				uint16_t highbyte = GPS_getByte() << 8;
				uint16_t lowbyte = GPS_getByte();
				uint16_t len = highbyte | lowbyte;
				gpsmsg.payload_length = len;

				if(gpsmsg.payload_length > MAXPAYLOADLEN) { //catch early
						GPSstatus.payload_length_error++;
						state = GETSTART;
						break;
				}
				else {
					gpsmsg.calculated_checksum = 0; //init to 0
					state = GETPAYLOAD;
				}
		}
		break;
	case GETPAYLOAD:
		//wait for payload_length-1 bytes;
		if(GPS_available() > (gpsmsg.payload_length-1)){
			for(uint16_t i = 0;i < gpsmsg.payload_length;i++){
				uint8_t incomingbyte = GPS_getByte(); //take 1 byte from GPS buffer
				gpsmsg.payload[i] = incomingbyte;
				gpsmsg.calculated_checksum ^= incomingbyte;
			}
			state = GETCHECKSUM;
		}
		break;
	case GETCHECKSUM:
		//wait for 1 byte
		if(GPS_available() > 0){
			gpsmsg.received_checksum = GPS_getByte();
			state=GETTRAILER;
		}
		break;
	case GETTRAILER:
		//wait for 2 bytes
		if(GPS_available() > 1){
				uint8_t trailer_rec1 = GPS_getByte();
				uint8_t trailer_rec2 = GPS_getByte();

				if (trailer_rec1 != TRAILER1)
				{
					GPSstatus.t1_error++;
					state = GETSTART;
					break;
				}
				else if (trailer_rec2 != TRAILER2)
				{
					GPSstatus.t2_error++;
					state = GETSTART;
					break;
				}
				state = VALIDATE;
				break;//while
		}
		break;//case
	case VALIDATE:
		//todo: timestamp/offset

		//validate checksum
		if(gpsmsg.calculated_checksum == gpsmsg.received_checksum) {
			GPSstatus.payloadOK++;
			//handle message
			HandlePayload(gpsmsg.payload,gpsmsg.payload_length);
		}
		else {
			GPSstatus.chksum_error++;
		}
		state = GETSTART;
		break;
	}

	//non-blocking (1Hz copy)
	if(FLAG_OKTOCOPYNAVDATA){
		if ( xSemaphoreTake( mutex_NAVDATA, (BaseType_t)10)== pdTRUE){
			NAVdata = tmpNAVdata;
			FLAG_OKTOCOPYNAVDATA = 0;
		xSemaphoreGive(mutex_NAVDATA);}
	}

}//end GPSprocess


//convert uint32 to two's compliment (S32)
int32_t s32(uint32_t fourbytes){
	return -(fourbytes & 0x80000000) | (fourbytes & 0x7fffffff);
}

//convert lat & lon s32 in 2C to float in proper GPS format
float convert2float(uint32_t fourbytes) {
	return s32(fourbytes)*.0000001;
}

uint8_t getFix(void){
	uint8_t data = 0;
	data = NAVdata.fixmode;
	return data;
}

uint8_t getNumSats(void){
	uint8_t data = 0;
	data = NAVdata.numsv;
	return data;
}

//in decimal degrees
float getLatitude(void) {
	float data = 0.0;
	data = NAVdata.latitude*.0000001;
	return data;

}

float getLongitude(void) {
	float data = 0.0;
	data = NAVdata.longitude*.0000001;
	return data;
}

//altitude above sealevel
float getAltitudeSealvl(void) {
	float alt = 0.0;
	alt = NAVdata.altitude_sealvl*.01;
	return alt;
}

uint32_t getTOW(void){
	uint32_t tow = 0;
	tow = NAVdata.gnss_time_of_week;
	return tow;
}

//Calculates msTicks clock error. Called in 1PPS external interrupt handler in main.c
void GPS_ppsHandler(uint32_t cur_msTicks){
	static uint8_t PPSINIT = 0;
	static uint32_t nextPPS = 0;

	tmp_PPStimestamp = cur_msTicks;

	if(PPSINIT) {
		int32_t error = cur_msTicks - nextPPS;

		//ignore larger errors due to Loss of Signal and nextPulse sync on startup
		if((error < 700) || (error > -700)) {
			GPSstatus.msTicks_error += error;
		}
	}

	nextPPS = cur_msTicks + 1000; //save new next
	PPSINIT = 1;
}

//returns a copy of the most recent valid NAV data for other modules to use
uint8_t getGPS(GPS_pub* myGPS){

	if ( xSemaphoreTake( mutex_NAVDATA, portMAX_DELAY) == pdTRUE){//ok to nest locks in freertos
	myGPS->fixmode = getFix();
	myGPS->numsv = NAVdata.numsv;
	myGPS->latitude = getLatitude();
	myGPS->longitude = getLongitude();
	myGPS->gnss_time_of_week = getTOW();
	myGPS->altitude_sealvl = getAltitudeSealvl();
	GPSstatus.numNAVmsgs++;
	xSemaphoreGive(mutex_NAVDATA);}

	return 1;
}



