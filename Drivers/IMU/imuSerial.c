/*
 * imuSerial.c
 *
 *  Created on: Jan 15, 2023
 *      Author: Bastien
 */

#include <string.h>

#include "imuSerial.h"
#include "imu_RegisterMap.h"

#include "trace.h"

/*** Variables ***/
static I2C_HandleTypeDef* i2c;

/*** Functions ***/
static uint8_t _processHalStatus(HAL_StatusTypeDef status);


/**
 * Init i2c transmission line
 * @param i2cInstance possible values : I2C1, I2C3 (defined in 'stm32l432xx.h')
 */
void imuSerial_Init(I2C_HandleTypeDef* hi2c)
{
	i2c = hi2c;
}

void imuSerial_Deinit(void)
{
	i2c = NULL;
}

uint8_t imuSerial_writeReg(const uint8_t addr, const uint8_t reg, const uint8_t data)
{
	uint8_t status = 0;

	uint8_t buff[2] = { reg, data };
	status = HAL_I2C_Master_Transmit(i2c, addr, buff, 2, MPU9250_TIMEOUT_ms);
	if(status != HAL_OK)
		return _processHalStatus(status);

	return 0;
}

uint8_t imuSerial_writeRegBurst(const uint8_t addr, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer)
{
	uint8_t status = 0;

	uint8_t buff[1 + nbRegs];
	buff[0] = startReg;
	memcpy(&buff[1], dataBuffer, nbRegs);
	status = HAL_I2C_Master_Transmit(i2c, addr, buff, 1 + nbRegs, MPU9250_TIMEOUT_ms);
	if(status != HAL_OK)
		return _processHalStatus(status);

	return 0;
}

/*
 * Read register and store it to data pointer
 * Output status :
 * 0 ok
 * 1 timeout
 * 2 error
 */
uint8_t imuSerial_readReg(const uint8_t addr, const uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_TIMEOUT;

	uint8_t buff = reg;
	status = HAL_I2C_Master_Transmit(i2c, addr, &buff, 1, MPU9250_TIMEOUT_ms);
	if(status != HAL_OK)
		return _processHalStatus(status);

	status = HAL_I2C_Master_Receive(i2c, addr, data, 1, MPU9250_TIMEOUT_ms);
	if(status != HAL_OK)
		return _processHalStatus(status);

	return 0;
}

/** Read multiple bytes from device.
    @param startReg First register regAddr to read from
    @param nbRegs Number of bytes to read
    @param dataBuffer Buffer to store read data in
    @return mpu9250 status
*/
uint8_t imuSerial_readRegBurst(const uint8_t addr, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer)
{
	HAL_StatusTypeDef status = HAL_TIMEOUT;

	uint8_t buff = startReg;
	status = HAL_I2C_Master_Transmit(i2c, addr, &buff, 1, MPU9250_TIMEOUT_ms);
	if(status != HAL_OK)
		return _processHalStatus(status);

	status = HAL_I2C_Master_Receive(i2c, addr, dataBuffer, nbRegs, MPU9250_TIMEOUT_ms);
	if(status != HAL_OK)
		return _processHalStatus(status);

	return 0;
}


/*** Private function definitions ***/

/***
 * Process status, convert it to MPU9250 status and return
 * 1 = timeout
 * 2 = error
 */
uint8_t _processHalStatus(HAL_StatusTypeDef status)
{
	uint32_t error = 0;

	switch(status)
	{
	default:
	case HAL_TIMEOUT:
		PRINT("timeout I2C\n");
		return 1;

	case HAL_ERROR:
		error = HAL_I2C_GetError(i2c);
		PRINT("error I2C: %d\n", error);
		return 2;
	}
}

