/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "GPS.h"
//#include "stm32f4xx_hal_i2c.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId GPSTaskHandle;
GPS_pub myGPSPUB;

uint16_t rx = 0;
volatile uint8_t RHdata[2] = {0,0};
volatile uint8_t TEMPdata[2] = {0,0};

#define DEVICESI 0x40
uint8_t addr = DEVICESI<<1;
float tempc;
float RHpct;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void GPSProcessTask(void const * argument);//handles GPS processing
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(GPSTask, GPSProcessTask, osPriorityNormal, 0, 128); //temporary
  //GPSTaskHandle = osThreadCreate(osThread(GPSTask), NULL);



  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/*
uint8_t I2C_read8(uint8_t reg){
	uint8_t data;
	I2C_GenerateSTART(I2C1, ENABLE);
}*/
/*
void I2C_get16(uint8_t address, uint8_t reg, uint8_t* pDataBuff){

	//wait for device
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) { }

	//request data at register reg
	while(HAL_I2C_Master_Transmit_IT(&hi2c1, address, (uint8_t*)&reg, 1)!= HAL_OK){
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
				  {
					while(1){}
					//Error_Handler();
				  }
		}

	//wait for device ready
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) { }

	while(HAL_I2C_Master_Receive_IT(&hi2c1, address, pDataBuff, 2)!= HAL_OK) {
			  // Error_Handler() function is called when Timout error occurs.
			  //	 When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
			 //	 Master restarts communication
		  if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)	  {
				  while(1){}
				///Error_Handler();
		}
	}
}
*/
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOD,LD6_Pin);

	getGPS(&myGPSPUB);
    osDelay(1500);

    //0xF5 ask RH no hold
    //0xE0 read temp from prev RH
    uint8_t SIREG_RH = 0xE5;
    uint8_t SIREG_TEMP = 0xE0;

    //getRH
    I2C_get16(addr,SIREG_RH, &RHdata);

    //calculate Relative humidity (as a percentage)
    uint16_t RHcode = (uint16_t)RHdata[0] << 8 | (uint16_t)RHdata[1];
    RHpct = (125.0*(float)(RHcode)/65536.0) - 6.0;

    //get TempC from previous RH measurement
    I2C_get16(addr,SIREG_TEMP, &TEMPdata);

    //calc tempC
    uint16_t tempcode = (uint16_t)TEMPdata[0] << 8 | (uint16_t)TEMPdata[1];
    tempc = (175.72*(float)(tempcode)/65536.0) - 46.85;

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void GPSProcessTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

   //check USART2queue for incoming GPS RHdata and handle it
	processGPS();

   //HAL_GPIO_TogglePin(GPIOD,LD5_Pin);
   //osDelay(2000);
   }
  /* USER CODE END 5 */
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
