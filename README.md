**NYUSat**  
This repository is for the OBC and Mission Science Payload on a 1U CubeSAT in developement at New York University. This software is developed for the STM32F407VTG ARM Cortex-M microcontroller controlling the scientific instruments and data collection on-board the satellite.

A full description and breakdown of the project can be found at: http://www.easonrobotics.com/?portfolio=nyu-cubesat

NYUSAT Team members: Abhimanyu Ghosh, Eason Smith, Matt Cocca, Danny Chiang, Dmytro Moyseyev

OBC/Payload embedded lead SW dev: Eason Smith (Eason@EasonRobotics.com)

An earlier version of this repositry that is no longer being developed can be found here:https://github.com/EasonNYC/NYUSat-old

HW Requirements:
STM32F407VGT6 ARM Cortex-M4

Currently Supported:  
Venus638FLPx (GPS)  
Si7021 (Humidty and Temerature)  
Geiger (Radiation)  
  
Currently Under development:  
CAN Tranciever
MPU9250 (Position, Acceleration, Velocity)  
BMP280 (Pressure, Temperature)  

SW Requirements:
STMCubemx / HAL / CMSIS library
FreeRTOS9 [non-stmcubemx edition]

