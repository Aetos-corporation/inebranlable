/*
 * xbee.c
 *
 *  Created on: Jan 6, 2023
 *      Author: Bastien
 */
#include <string.h>
#include <stdio.h>
#include <xbee/fifo.h>
#include <xbee/xbee.h>
#include <xbee/xbee_serial.h>
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include "log/frame.h"
#include "trace/trace.h"

#include "system/system.h"

extern uint8_t initFlags;
extern osThreadId xbeeTaskHandle;

/*** Private functions ***/
uint8_t _EnterCmdMode(void);
uint8_t _ExitCmdMode(void);
uint8_t _allocateFrame(uint32_t nbBytes);

void _resetFrame(void);
void _processStatus(uint8_t status);


/*** Variables ***/
uint8_t isCmdMode;		 //Boolean, is command mode active
uint8_t enteringCmdMode;

static uint8_t _frame[TX_FRAME_SIZE];
static uint32_t _size;

static Fifo_t frameFifo;

/*
 * @brief FreeRTOS Task
 * Setup : config xbee module and start receiving
 * Loop : Process incoming messages
 */
void StartXbeeTask(void const * argument)
{
	if(xbee_init(&huart1) != 0)
		vTaskDelete(xbeeTaskHandle);

	FifoInit(&frameFifo, sizeof(genericFrame_t), 100);

	sys_setInitFlag(SYS_MASK_XBEE);

	PRINT("XBee Start Task");

//Loop
	for(;;)
	{
		xbee_process();
		osDelay(10);
	}
}

// Initialiser l'interface de communication avec le module XBee
// Return 0 if ok,
//		  1 if else
uint8_t xbee_init(UART_HandleTypeDef* uartHandle)
{
	uint8_t ret = 0;

	PRINT("Init xbeeSerial...");
	xbeeSerial_Init(uartHandle);

	/*
	enteringCmdMode = 0;
	_resetFrame();

	_EnterCmdMode();

	if(ret != 0)
		return ret;

	// Configuration PAN ID
	xbee_send_at_command("ID", "30");
	ret = xbee_wait_for_AT_response();
	_processStatus(ret);
	if(ret != 0)
		return ret;

	// Configuration adresse de destination du second module XBee
	xbee_send_at_command("DH", "13A200");
	ret = xbee_wait_for_AT_response();
	_processStatus(ret);
	if(ret != 0)
		return ret;

	xbee_send_at_command("DL", "420D3B79");
	ret = xbee_wait_for_AT_response();
	_processStatus(ret);
	if(ret != 0)
		return ret;

	_ExitCmdMode();
*/

	return ret;
}

//Main process du module xbee
//Devrait être appelé toutes les 100ms
void xbee_process(void)
{
    if(IsFifoEmpty(&frameFifo)){
    	return;
    }
	genericFrame_t frame;
	uint8_t buffer[256];
	uint32_t bufferSize = 0;
	memset(buffer, 0x0, 256);

	FifoPop(&frameFifo, &frame);
	serialize(frame, buffer, &bufferSize);

//	PRINT("[Xbee] Sending frame: \n\r");
//	PRINT(" >func:     0x%02X\n\r", frame.codeFunc);
//	PRINT(" >mode:     0x%02X\n\r", frame.mode);
//	PRINT(" >dataSize: 0x%02X\n\r", frame.dataSize);
//	PRINT(" >data:     ");
//	for (int i = 0; i < bufferSize; i++)
//	    PRINT("0x%02X ", buffer[i]);
//	PRINT("\n\r\n\r");

	taskENTER_CRITICAL();
	xbeeSerial_Transmit(buffer, bufferSize);
    taskEXIT_CRITICAL();

	frameDelete(&frame); //Free Memory
}

// Copy command to frame buffer
// User must execute 'xbee_wait_for_AT_response' to send the issued AT request
// Return 0 if ok
//		  1 if buffer overflow
uint8_t xbee_send_at_command(const char *command, const char* param)
{
	if(!isCmdMode)
		_EnterCmdMode();

	// add comma if daisy chained commands
	if(_size != 0)
	{
		if(_allocateFrame(1))
			return 1;
		_frame[_size++] = ',';
	}

	if(_allocateFrame(4))
		return 1;

	memcpy(&_frame[_size], "AT", 2);
	_size += 2;
	memcpy(&_frame[_size], command, 2);
	_size += 2;

	uint8_t i = 0;
	uint8_t currByte = param[i];
	while(currByte != '\0')
	{
		if(_allocateFrame(1))
			return 1;

		_frame[_size++] = currByte;
		currByte =  param[++i];
	}

	PRINT("Command issued : AT %s %s\n", command, param); //Log

	return 0;
}

// Send all issued at command
// Wait for 'OK\r' response
// Return   0 if ok
//		    1 if timeout
//		    2 if response incorrect
uint8_t xbee_wait_for_AT_response(void)
{
	uint32_t startTime, currTime;

	if(!isCmdMode)
		return 0;

	//If in command mode
	if(!enteringCmdMode)
	{
		if(_allocateFrame(1))
			return 1;

		PRINT("Sending frame  : %.*s\n", _size, _frame); //log _frame. '\r' is not printed
		PRINT("...");
		_frame[_size++] = '\r';  //wrap up _frame

		xbeeSerial_Transmit(_frame, _size);  //Send _frame
	}

	//Check timeout
	startTime = HAL_GetTick();
	while(!xbeeSerial_isMessageReceived())
	{
		currTime = HAL_GetTick();
		//wait time over 1 second
		if((currTime - startTime) > 2000)
		{
			//Timeout, module failed to respond
			_resetFrame();
			return 1;
		}
	}

	//Get response
	uint8_t correctBuffer[3] = "OK\r";
	uint32_t rxBufferSize = xbeeSerial_getBufferSize();
	uint8_t rxBuffer[rxBufferSize];

	xbeeSerial_getBuffer(rxBuffer);
	if(memcmp(rxBuffer, correctBuffer, 3) != 0)
	{
		//Response is incorrect
		_resetFrame();
		return 2;
	}

	_resetFrame();
	return 0;
}

//Byte receive callback in transparent mode
//User should implement this
__weak void xbee_byteRcvCallback(const uint8_t byte)
{
	PRINT("%c", byte);
	if(byte == '\r')
		PRINT("\n");
}

void xbee_sendFrame(const genericFrame_t frame)
{
	FifoPush(&frameFifo, &frame);
}

/*** Private functions ***/


//Enter command mode function.
//Return 0 if OK, 1 else
uint8_t _EnterCmdMode(void)
{
	uint8_t ret = 1;
	uint8_t cmd[3] = "+++";

	//We need to set CmdMode in order for callback to work
	//Variable is reset later if fail
	isCmdMode = 1;
	enteringCmdMode = 1;

	xbeeSerial_Transmit(cmd, 3);
	ret = xbee_wait_for_AT_response();
	_processStatus(ret);
	if(ret != 0)
		isCmdMode = 0;
	else
		PRINT("  - CMD MODE\n");

	enteringCmdMode = 0;

	return ret;
}

uint8_t _ExitCmdMode(void)
{
	uint8_t ret = 1;
	
	isCmdMode = 1;

	xbee_send_at_command("CN", "");
	ret = xbee_wait_for_AT_response();
	_processStatus(ret);
	if(ret == 0)
	{
		ret = 0;
		isCmdMode = 0;
		PRINT("  - TRANSPARENT MODE\n");
	}

	return ret;
}

void _resetFrame(void)
{
	memset(_frame, 0, TX_FRAME_SIZE);
	_size = 0;
}

//Alloue un certain nombre de bytes dans la frame.
//Comme on rempli le buffer de données variable,
//permet de checker que le buffer n'overflow pas.
//Return 1 if _frame overflow
//		 0 else
uint8_t _allocateFrame(uint32_t nbBytes)
{
	uint8_t ret = 0;
	if((_size+nbBytes) > TX_FRAME_SIZE) //check size
	{
		//buffer overflow, erase _frame and return
		_resetFrame();
		ret = 1;
	}
	return ret;
}

void _processStatus(uint8_t status)
{
	switch(status)
	{
	case 0:
		PRINT("done\n\n");
		break;

	default:
	case 1:
		PRINT("/!\\ Timeout, communication failed!\n");
		break;

	case 2:
		PRINT("/!\\ Bad response, communication failed!\n");
		break;
	}
}
