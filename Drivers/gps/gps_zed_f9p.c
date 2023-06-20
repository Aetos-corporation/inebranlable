/*
 * gps_zed_f9p.c
 *
 *  Created on: 8 mars 2023
 *      Author: GatienJost
 */


#include "gps/gps_zed_f9p.h"
#include "usart.h"
#include "trace/trace.h"
#include <system/system.h>
#include <log/log.h>
#include "cmsis_os.h"

// - Variables globales - //
GPS gps_F9P;


// - Compteurs utilisés en cas de Debug - //

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


void GPS_Init(GPS* gps, UART_HandleTypeDef* huart, USART_TypeDef* uart, uint32_t baudrate){
	gps->huart = huart;
	GPS_Init_Uart(gps, uart, baudrate);

	gps->com.GPS_UART_Buffer_Index = 0;

	// Initialisation des flags de r�ception
	gps->data.receivedPositionFlag = false;

	gps->data.fixed = false;

	HAL_UART_Receive_IT(huart, &gps->com.GPS_UART_Buffer, 1);
}

void GPS_Init_Uart(GPS* gps, USART_TypeDef* uart, uint32_t baudrate){
	  huart2.Instance = uart;
	  huart2.Init.BaudRate = baudrate;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(gps->huart) != HAL_OK){
		GPS_Error_Handler();
	}
}

void GPS_UART_Handler(GPS* gps){
	gps->com.GPS_UART_RX_Chars[gps->com.GPS_UART_Buffer_Index] = gps->com.GPS_UART_Buffer;
	gps->com.GPS_UART_Buffer_Index++;

	if (gps->com.GPS_UART_Buffer == '\n'){
		gps->com.GPS_UART_RX_Chars[gps->com.GPS_UART_Buffer_Index] = '\0';
		gps->com.GPS_UART_Buffer_Index = 0;

		if ((gps->com.GPS_UART_RX_Chars[3] == 'G') && (gps->com.GPS_UART_RX_Chars[4] == 'G') && (gps->com.GPS_UART_RX_Chars[5] == 'A')){
			memcpy(gps->com.GPS_GGA_Buffer_Chars, gps->com.GPS_UART_RX_Chars, sizeof(gps->com.GPS_UART_RX_Chars));
			gps->com.GGAFrameReceived = true;
			DEBUG_CountGGA++;
		}

		else if ((gps->com.GPS_UART_RX_Chars[3] == 'Z') && (gps->com.GPS_UART_RX_Chars[4] == 'D') && (gps->com.GPS_UART_RX_Chars[5] == 'A')){
			memcpy(gps->com.GPS_ZDA_Buffer_Chars, gps->com.GPS_UART_RX_Chars, sizeof(gps->com.GPS_UART_RX_Chars));
			gps->com.ZDAFrameReceived = true;
			DEBUG_CountZDA++;
		}

		else if ((gps->com.GPS_UART_RX_Chars[3] == 'V') && (gps->com.GPS_UART_RX_Chars[4] == 'T') && (gps->com.GPS_UART_RX_Chars[5] == 'G')){
			memcpy(gps->com.GPS_VTG_Buffer_Chars, gps->com.GPS_UART_RX_Chars, sizeof(gps->com.GPS_UART_RX_Chars));
			gps->com.VTGFrameReceived = true;
			DEBUG_CountVTG++;
		}

		else{
			memset(gps->com.GPS_UART_RX_Chars, '\0', sizeof(gps->com.GPS_UART_RX_Chars));		// Amélioration : passer sur le stockage dans une FiFo
			//gps->com.NMEAFrameReceived = true;
		}

		DEBUG_CountFrames++;
	}

	if (gps->com.GPS_UART_Buffer_Index > 100){
		// - Erreur - //
		memset(gps->com.GPS_UART_RX_Chars, '\0', sizeof(gps->com.GPS_UART_RX_Chars));
		gps->com.GPS_UART_Buffer_Index = 0;
	}

	HAL_UART_Receive_IT(gps->huart, &gps->com.GPS_UART_Buffer, 1);
}

bool GPS_Parse_ZDA_Frame (GPS* gps)
{
	DEBUG_CountParsedZDA++;
	struct minmea_sentence_zda zda;

	minmea_parse_zda(&zda, gps->com.GPS_ZDA_Buffer_Chars);

	gps->data.time.tm_sec = zda.time.seconds;
	gps->data.time.tm_min = zda.time.minutes;
	gps->data.time.tm_hour = zda.time.hours;

	gps->data.time.tm_mday = zda.date.day;
	gps->data.time.tm_mon = zda.date.month;
	gps->data.time.tm_year = zda.date.year;

	minmea_gettime(&gps->data.timestamp, &zda.date, &zda.time);

	gps->data.date_release_time = HAL_GetTick();

	return true;
}

bool GPS_Parse_GGA_Frame (GPS* gps)
{
	DEBUG_CountParsedGGA++;
	struct minmea_sentence_gga gga;
	minmea_parse_gga(&gga, gps->com.GPS_GGA_Buffer_Chars);

	gps->data.longitude = minmea_tocoord(&gga.longitude);
	gps->data.latitude = minmea_tocoord(&gga.latitude);

	gps->data.altitude = minmea_tofloat(&gga.altitude);
	gps->data.hdop = minmea_tofloat(&gga.hdop);
	gps->data.satellitesUsed = gga.satellites_tracked;

	gps->data.time.tm_sec = gga.time.seconds;
	gps->data.time.tm_min = gga.time.minutes;
	gps->data.time.tm_hour = gga.time.hours;

	//GPS Quality indicator:
	//0: Fix not valid
	//1: GPS fix
	//2: Differential GPS fix, OmniSTAR VBS
	//4: Real-Time Kinematic, fixed integers
	//5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK

	gps->data.fixed = (gga.fix_quality>=1); // Le GPS est positionné lorsque l'indicateur de qualité est supérieur ou égal à 1

	gps->data.receivedPositionFlag = true;

	if (gps->data.fixed){									// MàJ de la "fraicheur" de l'information que si la donnée est considérée comme fiable
		gps->data.pos_release_time = HAL_GetTick();
	}

	return true;
}

void GPS_Error_Handler(void){
	return;
}

void StartGpsTask(void const * argument){
	// - Initialisation des capteurs - //
	GPS_Init(&gps_F9P, &huart2, USART2, 38400);
	sys_setInitFlag(SYS_MASK_GPS);

	// Mettre le loop ici avec un while 1
	while (1){
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* Acquisition GPS */
		if (gps_F9P.com.GGAFrameReceived){
			gps_F9P.com.GGAFrameReceived = false;
			GPS_Parse_GGA_Frame(&gps_F9P);
			LOG_GPS_LATITUDE(gps_F9P.data.latitude);
			LOG_GPS_LONGITUDE(gps_F9P.data.longitude);
		}

		if (gps_F9P.com.ZDAFrameReceived){
			gps_F9P.com.ZDAFrameReceived = false;
			GPS_Parse_ZDA_Frame(&gps_F9P);
			LOG_GPS_LATITUDE(gps_F9P.data.latitude);
			LOG_GPS_LONGITUDE(gps_F9P.data.longitude);
		}

		osDelay(SENSOR_UPDATE_RATE);
	}
}

float GPS_getLatitude(void)
{
	return gps_F9P.data.latitude;
}

float GPS_getLongitude(void)
{
	return gps_F9P.data.longitude;
}

//Interruption sur UART2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2){
		GPS_UART_Handler(&gps_F9P);
	}
}
