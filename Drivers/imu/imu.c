/*
 * imu.c
 *
 *  Created on: 16 mai 2023
 *      Author: Bastien
 */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <cmsis_os.h>

#include "i2c.h"
#include "imu/imu.h"
#include "imu/imuRegisterMap.h"
#include "trace/trace.h"
#include "usart.h"
#include "system/system.h"

/* PINOUT
 * ****************
 * SDA -> PB7 -> D4
 * SCL -> PB6 -> D5
 */

/*** Private functions declarations (hidden from the user) ***/
// Serial read/write
uint8_t imu_writeReg(imuHandle_t* imuH, const uint8_t reg, const uint8_t data);
uint8_t imu_writeRegBurst(imuHandle_t* imuH, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer);
uint8_t imu_readReg(imuHandle_t* imuH, const uint8_t reg, uint8_t *data);
uint8_t imu_readRegBurst(imuHandle_t* imuH, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer);

// Read absolute values from sensor
uint8_t imu_Init(imuHandle_t* imuH, I2C_HandleTypeDef* hi2c, const uint8_t addr);
void imu_Deinit(imuHandle_t* imuH);
void imu_setAddr(imuHandle_t* imuH, const uint8_t addr);
void imu_calibrate(imuHandle_t* imuH);
void imu_updateRPY(imuHandle_t* imuH);

float imu_readRoll(imuHandle_t* imuH);
float imu_readPitch(imuHandle_t* imuH);
float imu_readYaw(imuHandle_t* imuH);
void imu_updateOffset(imuHandle_t* imuH);
void imu_updateRPY(imuHandle_t* imuH);


/*** Public functions implementation ***/

imuHandle_t imu;

void StartImuTask(void const * argument)
{
	// Wait for log to be initialized
	while( !sys_testInitFlag(SYS_MASK_XBEE))
			osDelay(1000);

	PRINT("Init IMU...");

	taskENTER_CRITICAL();
	if( imu_Init(&imu, &hi2c1, IMU_DEFAULT_ADDR) == 0 )
	{
		PRINT("done");
	}
	else
	{
		PRINT("  Connexion error!");
		while(1);
	}
	taskEXIT_CRITICAL();

	sys_setInitFlag(SYS_MASK_IMU);

	for(;;){
		imu_updateRPY(&imu);
//		PRINT("Roll: %.3f   Pitch: %.3f   Yaw: %.3f", imu.roll, imu.pitch, imu.yaw);
		osDelay(1000);
	}
}

uint8_t imu_Init(imuHandle_t* imuH, I2C_HandleTypeDef* hi2c, const uint8_t addr)
{
	uint8_t version = 0;

	imuH->hi2c = hi2c;
	imuH->addr = addr;

	imuH->roll = 0;
	imuH->pitch = 0;
	imuH->yaw = 0;

	imuH->rollOffset = 0;
	imuH->pitchOffset = 0;
	imuH->yawOffset = 0;

	// Test connexion imu
	if( imu_readReg(imuH, CMPS12_CMD_VER_REG, &version) != IMU_OK )
		return IMU_KO;
	PRINT("  Version: 0x%X", version);

//	imu_calibrate(imuH);
	imu_updateOffset(imuH);

	return IMU_OK;
}

void imu_Deinit(imuHandle_t* imuH)
{
	imuH->hi2c = NULL;
	imuH->addr = 0x0;
}

void imu_calibrate(imuHandle_t* imuH)
{
	uint8_t regCalibration = 0x0, sysCalibration = 0x0, gyroCalibration = 0x0, accCalibration = 0x0, magCalibration = 0x0;

	// Delete calibration profile to sensor memory
	imu_writeReg(imuH, CMPS12_CMD_VER_REG, 0xE0);			 // Send 1st byte
	osDelay(20);
	imu_writeReg(imuH, CMPS12_CMD_VER_REG, 0xE5);			 // Send 2nd byte
	osDelay(20);
	imu_writeReg(imuH, CMPS12_CMD_VER_REG, 0xE2);			 // Send 3rd byte
	osDelay(20);

	// Wait for calibration flags to set
	imu_readReg(imuH, CMPS12_CALIBRATION_STATE_REG, &regCalibration);
	PRINT("\r\n  Move the sensor to calibrate...\r\n");
	PRINT("  Gyro – Place module in stationary state.\r\n");
	PRINT("  Accelerometer – tilt the module to roughly 45 and 90 degrees on one axis.\r\n");
	PRINT("  Magnetometer – Perform random movements.\r\n");
	sysCalibration  = (regCalibration >> 6) & 0x03;
	gyroCalibration = (regCalibration >> 4) & 0x03;
	accCalibration  = (regCalibration >> 2) & 0x03;
	magCalibration  = (regCalibration) & 0x03;
	PRINT("  Calib Level:  all 0x%X   sys %d   gyro %d   acc %d   mag %d\r\n", regCalibration, sysCalibration, gyroCalibration, accCalibration, magCalibration);

	osDelay(1000 / portTICK_PERIOD_MS);

	while(regCalibration != 0xFF)
	{
		HAL_Delay(1000);
		imu_readReg(imuH, CMPS12_CALIBRATION_STATE_REG, &regCalibration);
		sysCalibration  = (regCalibration >> 6) & 0x03;
		gyroCalibration = (regCalibration >> 4) & 0x03;
		accCalibration  = (regCalibration >> 2) & 0x03;
		magCalibration  = (regCalibration) & 0x03;
		static int i = 0;
		PRINT("Calib %d", i++);
		//		PRINT("  Calib Level:  all 0x%X   sys %d   gyro %d   acc %d   mag %d\r\n", regCalibration, sysCalibration, gyroCalibration, accCalibration, magCalibration);
		osDelay(1000 / portTICK_PERIOD_MS);
	}
	PRINT("done\r\n");

	// Save calibration profile to sensor memory
	imu_writeReg(imuH, CMPS12_CMD_VER_REG, 0xF0);			// Send 1st byte
	HAL_Delay(20);
	imu_writeReg(imuH, CMPS12_CMD_VER_REG, 0xF5);			// Send 2nd byte
	HAL_Delay(20);
	imu_writeReg(imuH, CMPS12_CMD_VER_REG, 0xF6);			// Send 3rd byte
	HAL_Delay(20);
}

float imu_getRoll()
{
	return imu.roll;
}

float imu_getPitch()
{
	return imu.pitch;
}

float imu_getYaw()
{
	return imu.yaw;
}


/*** Private functions implementation ***/

uint8_t imu_writeReg(imuHandle_t* imuH, const uint8_t reg, const uint8_t data)
{
	// Build data frame
	uint8_t buff[2] = { reg, data };

	// Send data
	if( HAL_I2C_Master_Transmit(imuH->hi2c, imuH->addr, buff, 2, IMU_MAX_TIMEOUT) != HAL_OK )
		return IMU_KO;

	return 0;
}

uint8_t imu_writeRegBurst(imuHandle_t* imuH, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer)
{
	uint8_t buff[1 + nbRegs];

	// Build data frame
	buff[0] = startReg;
	memcpy(&buff[1], dataBuffer, nbRegs);

	// Send data
	if( HAL_I2C_Master_Transmit(imuH->hi2c, imuH->addr, buff, 1 + nbRegs, IMU_MAX_TIMEOUT) != HAL_OK )
		return IMU_KO;

	return IMU_OK;
}

uint8_t imu_readReg(imuHandle_t* imuH, const uint8_t reg, uint8_t *data)
{
	uint8_t buff = reg;

	// Send read instruction
	if( HAL_I2C_Master_Transmit(imuH->hi2c, imuH->addr, &buff, 1, IMU_MAX_TIMEOUT) != HAL_OK )
		return IMU_KO;

	// Read response
	if( HAL_I2C_Master_Receive(imuH->hi2c, imuH->addr, data, 1, IMU_MAX_TIMEOUT) != HAL_OK )
		return IMU_KO;

	return IMU_OK;
}

uint8_t imu_readRegBurst(imuHandle_t* imuH, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer)
{
	uint8_t buff = startReg;

	// Send read instruction
	if( HAL_I2C_Master_Transmit(imuH->hi2c, imuH->addr, &buff, 1, IMU_MAX_TIMEOUT) != HAL_OK )
		return IMU_KO;

	// Read response
	if( HAL_I2C_Master_Receive(imuH->hi2c, imuH->addr, dataBuffer, nbRegs, IMU_MAX_TIMEOUT) != HAL_OK )
		return IMU_KO;

	return 0;
}

float imu_readRoll(imuHandle_t* imuH)
{
	int8_t rollRaw;
	imu_readReg(imuH, CMPS12_ROLL_8B_REG, (uint8_t*)&rollRaw);
	return (double)rollRaw * 0.70866;  // +/-127  ->  +/-90
}

float imu_readPitch(imuHandle_t* imuH)
{
	int8_t pitchRaw;
	imu_readReg(imuH, CMPS12_PITCH_8B_REG, (uint8_t*)&pitchRaw);
	return (double)pitchRaw * 0.70866;  // +/-127  ->  +/-90
}

float imu_readYaw(imuHandle_t* imuH)
{
	uint8_t yawRaw[2] = {0x0, 0x0};
	imu_readRegBurst(imuH, CMPS12_COMPASS_16B_HIGH_REG, 2, yawRaw);
	return (double)( (uint16_t)((yawRaw[0]<<8) | (yawRaw[1]&0xFF)) ) / 10;
}

void imu_updateOffset(imuHandle_t* imuH)
{
	for(uint8_t i=0 ; i<10 ; i++)
	{
		imuH->rollOffset = imu_readRoll(imuH);
		imuH->pitchOffset = imu_readPitch(imuH);
		imuH->yawOffset = imu_readYaw(imuH);
	}

	PRINT("  Offsets: ");
	PRINT("    roll:  %.3f", imuH->rollOffset);
	PRINT("    pitch: %.3f", imuH->pitchOffset);
	PRINT("    yaw:   %.3f", imuH->yawOffset);
}

void imu_updateRPY(imuHandle_t* imuH)
{
	// read val
	imuH->roll = imu_readRoll(imuH);
	// remove offset
	if(imuH->roll > 0)
		imuH->roll -= imuH->rollOffset;
	else
		imuH->roll += imuH->rollOffset;
	// modulo 90
	if(imuH->roll > 90)
		imuH->roll -= 90;
	else if (imuH->roll <= -90)
		imuH->roll += 90;

	// read val
	imuH->pitch = imu_readPitch(imuH) + imuH->pitchOffset;
	// remove offset
	if(imuH->pitch > 0)
		imuH->pitch -= imuH->pitchOffset;
	else
		imuH->pitch += imuH->pitchOffset;
	// modulo 90
	if(imuH->pitch > 90)
		imuH->pitch -= 90;
	else if (imuH->pitch <= -90)
		imuH->pitch += 90;

	// read val + remove offset
	imuH->yaw = imu_readYaw(imuH) - imuH->yawOffset;
	// modulo 360
	if(imuH->yaw > 360)
		imuH->yaw -= 360;
}
