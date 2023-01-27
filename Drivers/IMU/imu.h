/*
 * mpu9250.h
 *
 *  Created on: Jan 14, 2023
 *      Author: Bastien
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

#include "i2c.h"


uint8_t imu_init(I2C_HandleTypeDef* hi2c);

void set_acc_gyro_to_calibration(void);

// Multiple axes at once
void imu_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void imu_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my,int16_t* mz);

// Accelerometer
void imu_getAcceleration(int16_t* x, int16_t* y, int16_t* z);
int16_t imu_getAccelerationX();
int16_t imu_getAccelerationY();
int16_t imu_getAccelerationZ();


// Gyroscope
void imu_getRotation(int16_t* x, int16_t* y, int16_t* z);
int16_t imu_getRotationX();
int16_t imu_getRotationY();
int16_t imu_getRotationZ();

// Magnetometer
void imu_getDirection(int16_t* x, int16_t* y, int16_t* z);
int16_t imu_getDirectionX();
int16_t imu_getDirectionY();
int16_t imu_getDirectionZ();

#endif /* IMU_H_ */
