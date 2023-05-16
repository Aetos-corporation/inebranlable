/*
 * gps_zed_f9p.h
 *
 *  Created on: 8 mars 2023
 *      Author: GatienJost
 */

#ifndef __GPS_ZED_F9P_H
#define __GPS_ZED_F9P_H

#include "time.h"
#include "string.h"
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "gps/nmea.h"

typedef struct
{
	uint8_t Class;
	uint8_t ID;
	uint16_t PayloadLength;
	uint8_t Payload[8];
	uint16_t Checksum;

} UBXFrame;

typedef struct
{
	uint8_t GPS_UART_RX_Chars[100];
	uint32_t GPS_UART_Buffer_Index;
	uint8_t GPS_UART_Buffer;

	char GPS_UART_Buffer_Chars[100];
	char GPS_GGA_Buffer_Chars[100];
	char GPS_ZDA_Buffer_Chars[100];
	char GPS_VTG_Buffer_Chars[100];

	bool VTGFrameReceived;
	bool GGAFrameReceived;
	bool ZDAFrameReceived;

} GpsSerialCommunication;

typedef struct
{
	float latitude;
	float longitude;
	float hdop;
	float altitude;
	bool fixed;
	struct tm time;
	struct timespec timestamp;				//valeurs en sec et ns disponibles
	uint32_t pos_release_time;
	uint32_t date_release_time;
	bool receivedPositionFlag;
	uint8_t satellitesUsed;
} GpsData;

typedef struct
{
	UART_HandleTypeDef* huart;
	GpsData data;
	GpsSerialCommunication com;
}	GPS;

void GPS_Init(GPS* gps, UART_HandleTypeDef* huart, USART_TypeDef* uart, uint32_t baudrate);
void GPS_Init_Uart(GPS* gps, USART_TypeDef* uart, uint32_t baudrate);
void GPS_Error_Handler(void);
void GPS_UART_Handler(GPS* gps);

bool GPS_Parse_NMEA_Frame (GPS* gps);
bool GPS_Parse_ZDA_Frame (GPS* gps);
bool GPS_Parse_GGA_Frame (GPS* gps);

void GPS_Compute_checksums(uint8_t* msg, int start, int stop, int length);

// - Variables globales - //
GPS gps_F9P;

#endif

