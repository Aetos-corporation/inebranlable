/*
 * imuRegisterMap.h
 *
 *  Created on: May 16, 2023
 *      Author: Bastien
 */

#ifndef IMU_IMUREGISTERMAP_H_
#define IMU_IMUREGISTERMAP_H_

// CMPS12 Register Map
// https://www.robot-electronics.co.uk/files/cmps12.pdf

#define CMPS12_CMD_VER_REG		0x00	// Command register (write) / Software version (read)
// Computed values
#define CMPS12_COMPASS_8B_REG			0x01	// From 0 to 255, for a full circle
#define CMPS12_COMPASS_16B_HIGH_REG		0x02	// From 0 (0 degrees) to 3599 (359.9 degrees)
#define CMPS12_COMPASS_16B_LOW_REG		0x03
#define CMPS12_PITCH_8B_REG				0x04	// In degrees from the horizontal plane (+/-90°)
#define CMPS12_ROLL_8B_REG				0x05	// In degrees from the horizontal plane (+/-90°)
// Raw magnetometer values
#define CMPS12_MAG_X_HIGH_REG			0x06
#define CMPS12_MAG_X_LOW_REG			0x07
#define CMPS12_MAG_Y_HIGH_REG			0x08
#define CMPS12_MAG_Y_LOW_REG			0x09
#define CMPS12_MAG_Z_HIGH_REG			0x0A
#define CMPS12_MAG_Z_LOW_REG			0x0B
// Raw accelerometer values
#define CMPS12_ACC_X_HIGH_REG			0x0C
#define CMPS12_ACC_X_LOW_REG			0x0D
#define CMPS12_ACC_Y_HIGH_REG			0x0E
#define CMPS12_ACC_Y_LOW_REG			0x0F
#define CMPS12_ACC_Z_HIGH_REG			0x10
#define CMPS12_ACC_Z_LOW_REG			0x11
// Raw gyroscope values
#define CMPS12_GYR_X_HIGH_REG			0x12
#define CMPS12_GYR_X_LOW_REG			0x13
#define CMPS12_GYR_Y_HIGH_REG			0x14
#define CMPS12_GYR_Y_LOW_REG			0x15
#define CMPS12_GYR_Z_HIGH_REG			0x16
#define CMPS12_GYR_Z_LOW_REG			0x17
// Temperature in degrees centigrade
#define CMPS12_TEMP_HIGH_REG			0x18
#define CMPS12_TEMP_LOW_REG				0x19
//BNO55 values
#define CMPS12_BOSCH_COMPASS_16B_HIGH_REG		0x1A	// From 0 to 5759, divide by 16 for degrees
#define CMPS12_BOSCH_COMPASS_16B_LOW_REG		0x1B
#define CMPS12_PITCH_16B_HIGH_REG				0x1C	// In degrees from the horizontal plane (+/-180°)
#define CMPS12_PITCH_16B_LOW_REG				0x1D
// Calibration
#define CMPS12_CALIBRATION_STATE_REG			0x1E


#endif /* IMU_IMUREGISTERMAP_H_ */
