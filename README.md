NYUSat
This repository is for the OBC and Mission Science Payload being actively developed at New York University as part of the first phase development of a 1U cube satellite. This software is developed for the STM32F407VTG ARM-CortexM microcontroller and controls the collection of scientific data and  the synchronization of processes which command and control the scientific instruments used on-board the satellite.

NYUSAT Team members: Abhimanyu Ghosh, Eason Smith, Matt Cocca, Danny Chiang, Dmytro Moyseyev

OBC/Payload lead SW dev: Eason Smith (Eason@EasonRobotics.com)

An earlier version of this repositry that is no longer being developed can be found here:https://github.com/EasonNYC/NYUSat-old

HW Requirements:
STM32F407VGT6

Currently Supported:
Venus638FLPx (GPS)
Under development:
Si7021 (Humidty and Temerature)
Geiger (Radiation)
MPU9250 (Position, Acceleration, Velocity)
BMP280 (Pressure, Temperature)


SW Requirements:
STMCubemx / HAL / CMSIS library
FreeRTOS9 [non-stmcubemx edition]

Developement Enviroment:
Eclipse Neon