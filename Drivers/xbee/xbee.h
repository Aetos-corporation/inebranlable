/*
 * xbee.h
 *
 *  Created on: Jan 6, 2023
 *      Author: Bastien
 */

#ifndef XBEE_XBEE_H_
#define XBEE_XBEE_H_

#include "stm32l4xx_hal.h"
#include "log/frame.h"

#define TX_FRAME_SIZE   100

#define TIMEOUT_ERROR	1
#define AT_CMD_ERROR	2

uint8_t xbee_init(UART_HandleTypeDef* huart);

void xbee_process(void);

//Serial Cmds
uint8_t xbee_send_at_command(const char *command, const char* param);
uint8_t xbee_wait_for_AT_response(void);
void xbee_byteRcvCallback(const uint8_t byte);

//Frame
void xbee_sendFrame(const genericFrame_t frame);


#endif /* XBEE_XBEE_H_ */
