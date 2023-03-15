/*
 * serial.c
 *
 *  Created on: Jan 6, 2023
 *      Author: Bastien
 */
#include <string.h>
#include <trace.h>

#include "main.h"
#include "xbee.h"
#include "xbeeSerial.h"


/*** Private function prototype ***/
void _ReceiveStart(void);
void _ReceiveStop(void);

/*** Variables ***/
static UART_HandleTypeDef* uart;
static uint8_t rxByte;
static uint8_t ringBuffer[RING_BUFFER_SIZE];
static uint32_t head, tail;
static uint8_t msgReceived; //Message received flag

extern uint8_t isCmdMode;	//Boolean, is command mode active. Global variable
extern uint8_t enteringCmdMode;


void xbeeSerial_Init(UART_HandleTypeDef* huart)
{
	uart = huart;
	head = tail = rxByte = 0;

	_ReceiveStart();
}

void xbeeSerial_DeInit(void)
{
	_ReceiveStop();

	uart = NULL;
}

/*
 * Copy received buffer to destination
 */
void xbeeSerial_getBuffer(uint8_t* destination)
{
	int32_t msgSize;
	if(head < tail)
	{
		//message is splitted in buffer, copy in two time
		msgSize = RING_BUFFER_SIZE - tail + head;
		uint8_t halfSize = RING_BUFFER_SIZE - tail;
		memcpy(destination, &ringBuffer[tail], halfSize);
		memcpy(&destination[halfSize], ringBuffer, head);
	}
	else
	{
		//Message is not splitted, regular copy
		msgSize = head - tail;
		memcpy(destination, &ringBuffer[tail], msgSize);
	}

	//Update index
	tail = head;
}

/*
 * get amount of received bytes
 */
uint32_t xbeeSerial_getBufferSize(void)
{
	uint32_t size;
	if(head < tail)
		size = RING_BUFFER_SIZE - tail + head;
	else
		size = head - tail;
	return size;
}

/*
 * UART transmit blocking mode
 */
void xbeeSerial_Transmit(const uint8_t* txBuffer, const uint32_t size)
{
	HAL_UART_Transmit(uart, txBuffer, size, 100);
}

uint8_t xbeeSerial_isMessageReceived(void)
{
	return msgReceived;
}

/*** Private functions ***/
void _ReceiveStart(void)
{
	head = 0;
	tail = 0;
	msgReceived = 0;

	memset(ringBuffer, 0, RING_BUFFER_SIZE);

	HAL_UART_Receive_IT(uart, &rxByte, 1);
}

void _ReceiveStop(void)
{
	HAL_UART_AbortReceive_IT(uart);
}


/*** Interrupt routines ***/

void xbeeSerial_rxCallback(void)
{
	if(isCmdMode | enteringCmdMode)
	{
		if(rxByte == 0xD) // rxByte == '\r'
			msgReceived = 1;

		ringBuffer[head] = rxByte;
		head = (head+1) % RING_BUFFER_SIZE; //update index
	}
	else
		xbee_byteRcvCallback(rxByte);

	HAL_UART_Receive_IT(uart, &rxByte, 1);
}
