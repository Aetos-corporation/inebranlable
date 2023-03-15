/*
 * xbee.c
 *
 *  Created on: Jan 6, 2023
 *      Author: Bastien
 */
#include <string.h>
#include <stdio.h>
#include <trace.h>

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include "xbee.h"
#include "xbeeSerial.h"

extern osThreadId xbeeTaskHandle;

/*** Private functions ***/
uint8_t _EnterCmdMode(void);
uint8_t _ExitCmdMode(void);
uint8_t _allocateFrame(uint32_t nbBytes);

void _ResetFrame(void);
void _processStatus(uint8_t status);


/*** Variables ***/
uint8_t isCmdMode;		 //Boolean, is command mode active
uint8_t enteringCmdMode;

static uint8_t _frame[TX_FRAME_SIZE];
static uint32_t _size;

/*
 * @brief FreeRTOS Task
 * Setup : config xbee module and start receiving
 * Loop : Process incoming messages
 */
void StartXbeeTask(void const * argument)
{
	PRINT("XBee Start Task\n");

	if(xbee_init(&huart1) != 0)
		vTaskDelete(xbeeTaskHandle);

	osDelay(1000);

//Loop
	for(;;)
	{
		xbee_process();
		osDelay(100);
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

	enteringCmdMode = 0;
	_ResetFrame();

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

	return ret;
}

//Main process du module xbee
//Devrait être appelé toutes les 100ms
void xbee_process(void)
{
	static uint32_t startTime = 0;
	uint32_t currTime = HAL_GetTick();

	//time gate, vérifie que 1sec est passée afin d'être non-bloquant
	if( (currTime - startTime) <= 1000 )
		return;
	else
		startTime = HAL_GetTick();

//	uint8_t msg[] = "Here!\n";
//	xbee_send_data(msg, 6);
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
			_ResetFrame();
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
		_ResetFrame();
		return 2;
	}

	_ResetFrame();
	return 0;
}

// Envoyer des données au module XBee en mode transparent
void xbee_send_data(const uint8_t *data, int length)
{
	if(isCmdMode)
		_processStatus(_ExitCmdMode());

	//TODO convert to ascii
	PRINT("%.s", length, data);

	xbeeSerial_Transmit(data, length); //Send to module
}

//Byte receive callback in transparent mode
//User should implement this
__weak void xbee_byteRcvCallback(const uint8_t byte)
{
	PRINT("%c", byte);
	if(byte == '\r')
		PRINT("\n");
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

void _ResetFrame(void)
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
		_ResetFrame();
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
