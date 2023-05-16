/*
 * log.c
 *
 *  Created on: 19 mars 2023
 *      Author: Bastien
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <xbee/xbee.h>
#include <xbee/xbee_serial.h>
#include "cmsis_os.h"
#include "task.h"
#include "log/frame.h"
#include "log.h"
#include "trace/trace.h"


void LOG_GPS_NORD(float val){
	LOG_SENSOR(CODE_FUNC_GPS_NORD, val);
}

void LOG_GPS_WEST(float val){
	LOG_SENSOR(CODE_FUNC_GPS_WEST, val);
}

void LOG_GIROUETTE(float val){
	LOG_SENSOR(CODE_FUNC_GIROUETTE, val);
}

void LOG_ROLL(float val){
	LOG_SENSOR(CODE_FUNC_ROLL, val);
}

void LOG_PITCH(float val){
	LOG_SENSOR(CODE_FUNC_PITCH, val);
}

void LOG_YAW(float val){
	LOG_SENSOR(CODE_FUNC_YAW, val);
}

void LOG_SAFRAN(float val){
	LOG_SENSOR(CODE_FUNC_PWM_SAFRAN, val);
}

void LOG_VOILE(float val){
	LOG_SENSOR(CODE_FUNC_PWM_VOILE, val);
}

void LOG_SENSOR(uint8_t codeFunc, float val){
	genericFrame_t frame;
	frameCreate(&frame); //Allocate Memory

	frame.codeFunc = codeFunc;
	frame.mode = 1;
	frame.dataSize = 4;
	memcpy(frame.data, (uint8_t*)&val, sizeof(float));
	LOG(frame);
}

//Fonctionne comme un printf
void LOG_INFO(const char *fmt, ...){
	va_list argp;
    va_start(argp, fmt);
	LOG_TEXT(fmt, argp, CODE_FUNC_LOG_INFO);
    va_end(argp);
}

void LOG_WARNING(const char *fmt, ...){
	va_list argp;
    va_start(argp, fmt);
//	LOG_TEXT(CODE_FUNC_LOG_WARN, fmt, argp);
    va_end(argp);
}

void LOG_ERROR(const char *fmt, ...){
	va_list argp;
    va_start(argp, fmt);
//	LOG_TEXT(CODE_FUNC_LOG_ERROR, fmt, argp);
    va_end(argp);
}

void LOG_TEXT(const char *fmt, va_list arg, uint8_t codeFunc){
	genericFrame_t frame;
	frameCreate(&frame); //Allocate Memory

	frame.codeFunc = codeFunc;
	frame.mode = 1;

	// build string
	char string[0xFF];
	uint16_t stringSize = 0;
	stringSize = vsprintf(string,fmt,arg);

	//Split string in multiple frames if too long
	uint32_t nbCharToSend = stringSize;
	while(nbCharToSend != 0)
	{
		if(nbCharToSend > MAX_DATA_SIZE)
		{
			memcpy(frame.data, &string[stringSize - nbCharToSend], MAX_DATA_SIZE);
			frame.dataSize = MAX_DATA_SIZE;
			nbCharToSend -= MAX_DATA_SIZE;
		}
		else
		{
			memcpy(frame.data, &string[stringSize - nbCharToSend], nbCharToSend);
			frame.dataSize = nbCharToSend;
			nbCharToSend = 0;
		}
		LOG(frame);
	}
}

void LOG(const genericFrame_t frame)
{
	xbee_sendFrame(frame);
}
