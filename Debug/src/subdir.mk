################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/GPS.c \
../src/_initialize_hardware.c \
../src/_write.c \
../src/adc.c \
../src/can.c \
../src/circarray.c \
../src/dma.c \
../src/freertos.c \
../src/gpio.c \
../src/i2c.c \
../src/main.c \
../src/spi.c \
../src/stm32f4xx_hal_msp.c \
../src/stm32f4xx_hal_timebase_TIM.c \
../src/stm32f4xx_it.c \
../src/system_stm32f4xx.c \
../src/tim.c \
../src/usart.c 

OBJS += \
./src/GPS.o \
./src/_initialize_hardware.o \
./src/_write.o \
./src/adc.o \
./src/can.o \
./src/circarray.o \
./src/dma.o \
./src/freertos.o \
./src/gpio.o \
./src/i2c.o \
./src/main.o \
./src/spi.o \
./src/stm32f4xx_hal_msp.o \
./src/stm32f4xx_hal_timebase_TIM.o \
./src/stm32f4xx_it.o \
./src/system_stm32f4xx.o \
./src/tim.o \
./src/usart.o 

C_DEPS += \
./src/GPS.d \
./src/_initialize_hardware.d \
./src/_write.d \
./src/adc.d \
./src/can.d \
./src/circarray.d \
./src/dma.d \
./src/freertos.d \
./src/gpio.d \
./src/i2c.d \
./src/main.d \
./src/spi.d \
./src/stm32f4xx_hal_msp.d \
./src/stm32f4xx_hal_timebase_TIM.d \
./src/stm32f4xx_it.d \
./src/system_stm32f4xx.d \
./src/tim.d \
./src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DOS_USE_SEMIHOSTING -DTRACE -DOS_USE_TRACE_SEMIHOSTING_STDOUT -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -I"../system/include/FreeRTOS" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/stm32f4xx_hal_msp.o: ../src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DOS_USE_SEMIHOSTING -DTRACE -DOS_USE_TRACE_SEMIHOSTING_STDOUT -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -I"../system/include/FreeRTOS" -std=gnu11 -Wno-missing-prototypes -Wno-missing-declarations -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/stm32f4xx_hal_msp.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


