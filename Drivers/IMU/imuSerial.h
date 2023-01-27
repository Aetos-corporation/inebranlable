/*
 * mpu9250Serial.h
 *
 *  Created on: Jan 14, 2023
 *      Author: Bastien
 */

#ifndef IMUSERIAL_H_
#define IMUSERIAL_H_

#include "i2c.h"

enum{
	MPU9250_ADDR = (0x68<<1),  // PA0 low = 0x68; PA0 high = 0x69
	MPU9250_TIMEOUT_ms = 1000,
	MPU9250_RING_BUFFER_SIZE = 100,
	MPU9250_WHO_AM_I_RESULT = 0x71
};

enum{
	AK8963_ADDR	= (0x0C<<1),
	AK8963_WHO_AM_I_RESULT = 0x48,

};

void imuSerial_Init(I2C_HandleTypeDef* hi2c);
void imuSerial_Deinit(void);


uint8_t imuSerial_writeReg(const uint8_t addr, const uint8_t reg, const uint8_t data);
uint8_t imuSerial_writeRegBurst(const uint8_t addr, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer);

uint8_t imuSerial_readReg(const uint8_t addr, const uint8_t reg, uint8_t *data);
uint8_t imuSerial_readRegBurst(const uint8_t addr, const uint8_t startReg, const uint8_t nbRegs, uint8_t* dataBuffer);


#endif /* IMUSERIAL_H_ */
