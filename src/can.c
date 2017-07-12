/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "can.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t mutex_CANcomm = NULL;



/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_1TQ;
  hcan1.Init.BS2 = CAN_BS2_1TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

	  mutex_CANcomm = xSemaphoreCreateMutex(); //create i2c1 mutex


  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}



void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

  }
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */


typedef union u32pub{
	uint32_t  u32;
	uint8_t b[4];
}u32pub;

typedef union s32pub{
	int32_t  f32;
	uint8_t b[4];
}s32pub;

typedef union f32pub{
	double  f32;
	uint8_t b[4];
}f32pub;

void publish2CAN(uint8_t ARBID, uint32_t timestamp, uint32_t data) {

	  u32pub t;
	  u32pub d;

	  t.u32 = timestamp;
	  d.u32 = data;
	  CanTxMsgTypeDef canmsg;
	  canmsg.Data

	  if ( xSemaphoreTake( mutex_CANcomm, portMAX_DELAY) == pdTRUE){

	  hcan1.pTxMsg->StdId = ARBID; //unique msg ID / priority
	  hcan1.pTxMsg->RTR = CAN_RTR_DATA; //broadcast
	  hcan1.pTxMsg->IDE = CAN_ID_STD;

	  hcan1.pTxMsg->DLC = 8; //msglen   (4bytes timestamp + 4bytes data)

	  //assemble the message
	  hcan1.pTxMsg->Data[0] = 0x6;
	  hcan1.pTxMsg->Data[1] = 0xFF;
	  	  hcan1.pTxMsg->Data[2] = 0xFF;
	  	  hcan1.pTxMsg->Data[3] = 0xFF;

	  	  hcan1.pTxMsg->Data[4] = 0xFF;
	  	  hcan1.pTxMsg->Data[5] = 0xFF;
	  	  hcan1.pTxMsg->Data[6] = 0xFF;
	  	  hcan1.pTxMsg->Data[7] = 0xFF;

	  /*
	  hcan1.pTxMsg->Data[0] = t.b[0];
	  hcan1.pTxMsg->Data[1] = t.b[1];
	  hcan1.pTxMsg->Data[2] = t.b[2];
	  hcan1.pTxMsg->Data[3] = t.b[3];

	  hcan1.pTxMsg->Data[4] = d.b[0];
	  hcan1.pTxMsg->Data[5] = d.b[1];
	  hcan1.pTxMsg->Data[6] = d.b[2];
	  hcan1.pTxMsg->Data[7] = d.b[3];

	  //for debug    todo: comment this out later


	  hcan1.pTxMsg->Data[0] = 0x6;
	  hcan1.pTxMsg->Data[1] = 0xFF;
	  hcan1.pTxMsg->Data[2] = 0xFF;
	  hcan1.pTxMsg->Data[3] = 0xFF;

	  hcan1.pTxMsg->Data[4] = 0xFF;
	  hcan1.pTxMsg->Data[5] = 0xFF;
	  hcan1.pTxMsg->Data[6] = 0xFF;
	  hcan1.pTxMsg->Data[7] = 0xFF;
	   */

	  if(HAL_CAN_Transmit(&hcan1, 100) != HAL_OK)
	  {
	    /* Transmition Error */
	   // Error_Handler();
	  }

	  if(HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY)
	  	  {
//	  	    return HAL_ERROR;
	  	  }
	  xSemaphoreGive(mutex_CANcomm);}//end CAN critical section

}




/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
