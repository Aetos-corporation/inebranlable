/*
 * xbee_core.h
 *
 *  Created on: Jan 6, 2023
 *      Author: Bastien
 */

#ifndef XBEES2C_XBEE_CORE_H_
#define XBEES2C_XBEE_CORE_H_

#include <stdint.h>

#include "stm32l4xx_hal.h"

#define RING_BUFFER_SIZE   	100
#define MIN_NB_BYTES_RX  	1


void xbeeSerial_Init(UART_HandleTypeDef* huart);
void xbeeSerial_DeInit(void);

void xbeeSerial_Transmit(const uint8_t* txBuffer, const uint32_t size);

uint8_t xbeeSerial_isMessageReceived(void);
uint32_t xbeeSerial_getBufferSize(void);
void xbeeSerial_getBuffer(uint8_t* destination);

void xbeeSerial_rxCallback(void);


#endif /* XBEES2C_XBEE_CORE_H_ */
