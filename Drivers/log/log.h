/*
 * log.h
 *
 *  Created on: 19 mars 2023
 *      Author: Bastien
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include <stdarg.h>

#include "log/frame.h"

#define CODE_FUNC_GPS_LONG   0x01
#define CODE_FUNC_GPS_LATI   0x02
#define CODE_FUNC_GIROUETTE  0x03
#define CODE_FUNC_ROLL       0x04
#define CODE_FUNC_PITCH      0x05
#define CODE_FUNC_YAW        0x06
#define CODE_FUNC_PWM_SAFRAN 0x07
#define CODE_FUNC_PWM_VOILE  0x08
#define CODE_FUNC_LOG_INFO   0x09
#define CODE_FUNC_LOG_WARN   0x10
#define CODE_FUNC_LOG_ERROR  0x11

void LOG_GPS_LONGITUDE(float val);
void LOG_GPS_LATITUDE(float val);
void LOG_GIROUETTE(float val);
void LOG_ROLL(float val);
void LOG_PITCH(float val);
void LOG_YAW(float val);
void LOG_SAFRAN(float val);
void LOG_VOILE(float val);

void LOG_INFO(const char *fmt, ...);
void LOG_WARNING(const char *fmt, ...);
void LOG_ERROR(const char *fmt, ...);

void LOG_SENSOR(uint8_t codeFunc, float val);
void LOG_TEXT(const char *fmt, va_list arg, uint8_t codeFunc);

void LOG(const genericFrame_t frame);

#endif /* INC_LOG_H_ */
