################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/gps/gps_zed_f9p.c \
../Drivers/gps/nmea.c 

OBJS += \
./Drivers/gps/gps_zed_f9p.o \
./Drivers/gps/nmea.o 

C_DEPS += \
./Drivers/gps/gps_zed_f9p.d \
./Drivers/gps/nmea.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/gps/%.o Drivers/gps/%.su: ../Drivers/gps/%.c Drivers/gps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Clement/Centrale/Projet Bateau/inebranlable-dev/Drivers" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-gps

clean-Drivers-2f-gps:
	-$(RM) ./Drivers/gps/gps_zed_f9p.d ./Drivers/gps/gps_zed_f9p.o ./Drivers/gps/gps_zed_f9p.su ./Drivers/gps/nmea.d ./Drivers/gps/nmea.o ./Drivers/gps/nmea.su

.PHONY: clean-Drivers-2f-gps

