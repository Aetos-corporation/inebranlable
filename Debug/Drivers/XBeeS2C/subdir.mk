################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/XBeeS2C/xbee.c \
../Drivers/XBeeS2C/xbeeSerial.c 

OBJS += \
./Drivers/XBeeS2C/xbee.o \
./Drivers/XBeeS2C/xbeeSerial.o 

C_DEPS += \
./Drivers/XBeeS2C/xbee.d \
./Drivers/XBeeS2C/xbeeSerial.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/XBeeS2C/%.o Drivers/XBeeS2C/%.su: ../Drivers/XBeeS2C/%.c Drivers/XBeeS2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-XBeeS2C

clean-Drivers-2f-XBeeS2C:
	-$(RM) ./Drivers/XBeeS2C/xbee.d ./Drivers/XBeeS2C/xbee.o ./Drivers/XBeeS2C/xbee.su ./Drivers/XBeeS2C/xbeeSerial.d ./Drivers/XBeeS2C/xbeeSerial.o ./Drivers/XBeeS2C/xbeeSerial.su

.PHONY: clean-Drivers-2f-XBeeS2C

