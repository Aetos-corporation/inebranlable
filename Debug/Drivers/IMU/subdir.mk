################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU/imu.c \
../Drivers/IMU/imuSerial.c \
../Drivers/IMU/imu_quaternionFilter.c 

OBJS += \
./Drivers/IMU/imu.o \
./Drivers/IMU/imuSerial.o \
./Drivers/IMU/imu_quaternionFilter.o 

C_DEPS += \
./Drivers/IMU/imu.d \
./Drivers/IMU/imuSerial.d \
./Drivers/IMU/imu_quaternionFilter.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU/%.o Drivers/IMU/%.su: ../Drivers/IMU/%.c Drivers/IMU/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/Prog/bateau/repos_github/inebranlable_imu/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IMU

clean-Drivers-2f-IMU:
	-$(RM) ./Drivers/IMU/imu.d ./Drivers/IMU/imu.o ./Drivers/IMU/imu.su ./Drivers/IMU/imuSerial.d ./Drivers/IMU/imuSerial.o ./Drivers/IMU/imuSerial.su ./Drivers/IMU/imu_quaternionFilter.d ./Drivers/IMU/imu_quaternionFilter.o ./Drivers/IMU/imu_quaternionFilter.su

.PHONY: clean-Drivers-2f-IMU

