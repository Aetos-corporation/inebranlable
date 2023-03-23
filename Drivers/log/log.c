/*
 * log.c
 *
 *  Created on: 19 mars 2023
 *      Author: Bastien
 */
#include <string.h>
#include <stdlib.h>

#include "cmsis_os.h"
#include "xbee.h"
#include "task.h"
#include "frame.h"
#include "log.h"
#include "trace.h"

#include "xbee_core.h"

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
	frame.dataSize = size;
	memcpy(frame.data, (uint8_t*)&val, sizeof(float));
	LOG(frame);
}

//Fonctionne comme un printf
void LOG_INFO(const char *fmt, ...){
	LOG_TEXT(CODE_FUNC_LOG_INFO, val, 4);
}

void LOG_WARNING(const char *fmt, ...){
	LOG_TEXT(CODE_FUNC_LOG_WARN, val, 4);
}

void LOG_ERROR(const char *fmt, ...){
	LOG_TEXT(CODE_FUNC_LOG_ERROR, val, 4);
}

void LOG_TEXT(uint8_t codeFunc, const char *fmt, ...){
	genericFrame_t frame;
	frameCreate(&frame); //Allocate Memory

	frame.codeFunc = codeFunc;
	frame.mode = 1;
	frame.dataSize = size;
	memcpy(frame.data, (uint8_t*)&val, sizeof(float));
	LOG(frame);
}

void LOG(const genericFrame_t frame)
{
	xbee_sendFrame(frame);
}
