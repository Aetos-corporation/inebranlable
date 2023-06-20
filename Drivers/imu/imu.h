/*
 * imu.h
 *
 *  Created on: 16 mai 2023
 *      Author: Bastien
 */
#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "i2c.h"

#define IMU_MAX_TIMEOUT		1000
#define IMU_DEFAULT_ADDR	0xC0

typedef enum {
	IMU_OK = 0,
	IMU_KO
} imuStatus_t;

typedef struct {
	I2C_HandleTypeDef* hi2c;
	uint8_t addr;

	// RPY values
	double roll;
	double pitch;
	double yaw;

	// RPY offset values
	double rollOffset;
	double pitchOffset;
	double yawOffset;
}imuHandle_t;

// Setup functions
float imu_getRoll();
float imu_getPitch();
float imu_getYaw();

#endif /* IMU_IMU_H_ */
