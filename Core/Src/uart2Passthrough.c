/*
 * uart2Passtrough.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Bastien
 */
#include "uart2Passthrough.h"
#include "usart.h"

#include "xbee.h"

static uint8_t rxByte;

void u2pt_StartReceive(void)
{
	HAL_UART_Receive_IT(&huart2, &rxByte, 1);
}

void u2pt_StopReceive(void)
{
	HAL_UART_AbortReceive_IT(&huart2);
}

void u2pt_rxCallback(void)
{
	xbee_send_data(&rxByte, 1);  //Send data to xbee
	HAL_UART_Receive_IT(&huart2, &rxByte, 1); //trigger new receive
}
