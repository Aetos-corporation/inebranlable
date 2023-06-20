################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/windSensor/windSensor.c 

OBJS += \
./Drivers/windSensor/windSensor.o 

C_DEPS += \
./Drivers/windSensor/windSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/windSensor/%.o Drivers/windSensor/%.su: ../Drivers/windSensor/%.c Drivers/windSensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/Prog/bateau/repos_github/inebranlable_dev/Drivers" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-windSensor

clean-Drivers-2f-windSensor:
	-$(RM) ./Drivers/windSensor/windSensor.d ./Drivers/windSensor/windSensor.o ./Drivers/windSensor/windSensor.su

.PHONY: clean-Drivers-2f-windSensor

