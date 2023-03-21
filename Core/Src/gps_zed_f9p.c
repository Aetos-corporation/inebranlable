/*
 * gps_zed_f9p.c
 *
 *  Created on: 8 mars 2023
 *      Author: GatienJost
 */


#include "gps_zed_f9p.h"

uint16_t DEBUG_InvalidCount = 0;
uint16_t DEBUG_UnknownCount = 0;
uint16_t DEBUG_CountFrames = 0;
uint16_t DEBUG_CountGGA = 0;
uint16_t DEBUG_CountRMC = 0;
uint16_t DEBUG_CountZDA = 0;
uint16_t DEBUG_CountVTG = 0;
uint16_t DEBUG_CountParsedGGA = 0;
uint16_t DEBUG_CountParsedRMC = 0;
uint16_t DEBUG_CountParsedZDA = 0;

uint8_t GPS_Buffer_ACK[50];
bool LoggingFrameUBX = false;

uint8_t enable_frame[30];

int f = 0;


void GPS_Init(GPS* gps, UART_HandleTypeDef* huart, USART_TypeDef* uart, uint32_t baudrate)
{
	gps->huart = huart;
	GPS_Init_Uart(gps, uart, baudrate);

	GPS_Set_bauds_to_115200(gps);

	gps->com.GPS_UART_Buffer_Index = 0;

	// Initialisation des flags de r�ception
	gps->data.receivedPositionFlag = false;
	gps->com.UBXFrameReceived = false;
	gps->com.NMEAFrameReceived = false;

	gps->GPSConfigured = false;
	gps->StateOfConfig = 0;
	gps->data.fixed = false;

	HAL_UART_Receive_IT(huart, &gps->com.GPS_UART_Buffer, 1);
}

void GPS_Set_bauds_to_115200(GPS* gps)
{
	uint8_t enable_115200[28] = {0XB5, 0X62, 0X06, 0X00, 0X14, 0X00, 0X01, 0X00, 0X00, 0X00, 0XD0, 0X08, 0X00, 0X00, 0X00, 0XC2, 0X01, 0X00, 0X07, 0X00, 0X03, 0X00, 0X00, 0X00, 0X00, 0X00, 0XC0, 0X7E};

	memset(enable_frame, '\0', sizeof(enable_115200));
	memcpy(enable_frame, enable_115200, sizeof(enable_115200));

	HAL_UART_Transmit(gps->huart, enable_frame, sizeof(enable_115200), 1000);
	GPS_Set_UART_Baudrate(gps, 115200);

	HAL_UART_Receive_IT(gps->huart, &gps->com.GPS_UART_Buffer, 1);
}
